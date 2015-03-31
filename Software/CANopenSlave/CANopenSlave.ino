#include <avr/io.h>
#include "canfestival.h"
#include "mcp_can.h"
#include "ObjDict.h"
#include "timerscfg.h"
#include "can.h"
#include "Arduino.h"
#include "SPI.h"
#include "DHT22.h"
#include <Wire.h> // TODO: replace by I2C library
#include <Adafruit_BMP085.h>
//#include <avr/wdt.h> 
#include "ApplicationMonitor.h"
#include <MsTimer2.h>
#include <BlinkPattern.h>
#include "error_state.h"
#include "rx_error_state.h"

// TODO: move all timers to timer0
// TODO: make hook for periodic measurements available
// TODO: add eeprom persistance

// CAN
MCP_CAN CAN(7);
static Message m = Message_Initializer;		// contain a CAN message

// this only works with optiboot v5.0a which is __not__ shipped with Arduino IDE 1.0.5!!
uint8_t resetFlags __attribute__ ((section(".noinit")));
void resetFlagsInit(void) __attribute__ ((naked)) __attribute__ ((section (".init0")));
// this is not a function, it is automatically executed at startup!
void resetFlagsInit(void)
{
  // save the reset flags passed from the bootloader
  __asm__ __volatile__ ("mov %0, r2\n" : "=r" (resetFlags) :);
}

Watchdog::CApplicationMonitor ApplicationMonitor;

#define LED_GREEN_INIT DDRD|= _BV(DDD4)
#define LED_GREEN_ON   PORTD|= _BV(PD4)
#define LED_GREEN_OFF  PORTD&= ~_BV(PD4)

#define LED_RED_INIT DDRD|= _BV(DDD3)
#define LED_RED_ON   PORTD|= _BV(PD3)
#define LED_RED_OFF  PORTD&= ~_BV(PD3)

// #define OUTPUT_MASK 0x78
#define SWITCH1_INIT DDRD|= _BV(DDD5)
#define SWITCH1_ON   PORTD|= _BV(PD5)
#define SWITCH1_OFF  PORTD&= ~_BV(PD5)
#define SWITCH1_STATE  (PIND&_BV(PIND5))

BlinkPattern RedBlink= BlinkPattern();
BlinkPattern GreenBlink= BlinkPattern();

void blink() {
  if(RedBlink.nextState())
    LED_RED_ON;
  else
    LED_RED_OFF;
  
  if(GreenBlink.nextState())
    LED_GREEN_ON;
  else
    LED_GREEN_OFF;
}

// Data wire is plugged into port 7 on the Arduino
// Connect a 4.7K resistor between VCC and the data pin (strong pullup)
#define DHT22_PIN A0

// Setup a DHT22 instance
DHT22 myDHT22(DHT22_PIN);
Adafruit_BMP085 bmp;
boolean bmpOK;

const float VoltageScale= 0.015702;
boolean underVoltageState= false;

const uint8_t CANopenErrReg_Generic= _BV(0);
const uint8_t CANopenErrReg_Current= _BV(1);
const uint8_t CANopenErrReg_Voltage= _BV(2);
const uint8_t CANopenErrReg_Temperature= _BV(3);
const uint8_t CANopenErrReg_Communication= _BV(4);
const uint8_t CANopenErrReg_DevProfile= _BV(5);
const uint8_t CANopenErrReg_Manufacturer= _BV(7);

const uint16_t CANopenSoftErr_WatchDog= 0x6000;
const uint16_t CANopenMonErr_UserRest= 0x8000;
const uint16_t CANopenSoftErr_BrownOut= 0x3000;
const uint16_t CANopenDevErr_DHTErrorCode= 0xFF00;
const uint16_t CANopenDevErr_BMPErrorCode= 0xFF10;

const uint16_t CANopenMonErr_TxWarn= 0x8100;
const uint16_t CANopenMonErr_TxAllBusy= 0x8101;
const uint16_t CANopenMonErr_TxPassive= 0x8102;
const uint16_t CANopenMonErr_TxErr= 0x8103;
const uint16_t CANopenMonErr_TxBusOff= 0x8104;

const uint16_t CANopenMonErr_RxWarn= 0x8200;
const uint16_t CANopenMonErr_RxPassive= 0x8201;
const uint16_t CANopenMonErr_RxOverFlow= 0x8202;

const uint16_t CANopenMonErr_UnderVoltage= 0x8300;

//uint32_t glitchDetector __attribute__ ((section (".noinit")));
//#define CANopenGlitchErrorCode 0x3000

tx_error_state_enum tx_error_state= tx_no_error;
tx_error_state_enum tx_last_error= tx_no_error;
rx_error_state_enum rx_error_state= rx_no_error;
rx_error_state_enum rx_last_error= rx_no_error;

void setTxErrorState(tx_error_state_enum e) {
  if(e>tx_error_state) {
    tx_error_state= e;
    switch(e) {
      case tx_warning:
        RedBlink.setPattern(BlinkPattern::Blink2);
        break;
      case tx_all_busy:
        RedBlink.setPattern(BlinkPattern::Blink31);
        break;
      case tx_passive:
        RedBlink.setPattern(BlinkPattern::Blink4);
        break;
      case tx_error:
        RedBlink.setPattern(BlinkPattern::Blink62);
        break;
      case tx_bus_off:
        RedBlink.setPattern(BlinkPattern::Blink8);
        break;
    }
  }
}

void resetTxErrorState() {
  tx_error_state= tx_no_error;
  RedBlink.setPattern(BlinkPattern::OFF);
}

void setRxErrorState(rx_error_state_enum e) {
  if(e>rx_error_state) {
    rx_error_state= e;
    switch(e) {
      case rx_warning:
        GreenBlink.setPattern(BlinkPattern::Blink2);
        break;
      case rx_passive:
        GreenBlink.setPattern(BlinkPattern::Blink4);
        break;
      case rx_overflow:
        GreenBlink.setPattern(BlinkPattern::Blink14);
        break;
    }
  }
}

void resetRxErrorState() {
  rx_error_state= rx_no_error;
  GreenBlink.setPattern(BlinkPattern::OFF);
}

unsigned char canSend(CAN_PORT notused, Message *m) {
  if(CAN.sendMsgBuf(m->cob_id, 0, m->rtr, m->len, m->data)==CAN_ALL_TX_BUSY) {
    if(CAN.checkTxError())
      setTxErrorState(tx_error);
    else
      setTxErrorState(tx_all_busy);
  } else {
    if(tx_error_state==tx_no_error) RedBlink.setPattern(BlinkPattern::Flash1);
  }
  return 0;
}


unsigned char canReceive(Message *m) {
    if(CAN.checkReceive())            // check if data coming
    {
      //      readMsgBufID(&(m->cob_id), &(m->len), m->data);
      CAN.readMsgBuf(&(m->len), m->data);
      m->cob_id= CAN.getCanId();
      m->rtr= 0; // m_nRtr;
      return 1;
    } else
      return 0;
}


unsigned char canChangeBaudRate_driver( CAN_HANDLE fd, char* baud) {
  return 1;
}

UNS32 readDHTCallback(CO_Data* d, const indextable *, UNS8 bSubindex) {
  static DHT22_ERROR_t last_error;
  DHT22_ERROR_t errorCode = myDHT22.readData();
  if(errorCode==DHT_ERROR_TOOQUICK) errorCode= DHT_ERROR_NONE;
  if(errorCode!=last_error) {
    EMCY_errorRecovered(&ObjDict_Data, CANopenDevErr_DHTErrorCode | last_error);
    last_error= DHT_ERROR_NONE;
  }
  
  // Serial.println("2s");
  if(errorCode==DHT_ERROR_NONE) {
    DHT22_Temperature= myDHT22.getTemperatureC();
    DHT22_Humidity= myDHT22.getHumidity();
  } else {
    if(!last_error) {
      last_error= errorCode;
      EMCY_setError(&ObjDict_Data, CANopenDevErr_DHTErrorCode | last_error, CANopenErrReg_Manufacturer, 0);
    }
    DHT22_Temperature= -100.0;
    DHT22_Humidity= -100.0;
    
  }
  return 0;
}

UNS32 readBMPPressureCallback(CO_Data* d, const indextable *, UNS8 bSubindex) {
  if(bmpOK)
    BMP_Pressure= bmp.readPressure();
  else
    BMP_Pressure= 0;
  
//  Serial.print("Pressure = ");
//  Serial.print(BMP_Pressure);
//  Serial.println(" Pa");
  
  return 0;
}

UNS32 readBMPTemperatureCallback(CO_Data* d, const indextable *, UNS8 bSubindex) {
  if(bmpOK)
    BMP_Temperature= bmp.readTemperature();
  else
    BMP_Temperature= -100.0;
//  Serial.print("Temperature = ");
//  Serial.print(BMP_Temperature);
//  Serial.println(" *C");
  
  return 0;
}

UNS32 readCounterCallback(CO_Data* d, const indextable *, UNS8 bSubindex) {
  Counter++;
  d->PDO_status[d->currentPDO].event_trigger= 1;
  return 0;
}

UNS32 rwLockCallback(CO_Data* d, const indextable *, UNS8 bSubindex) {
  if(Lock)
    while(1);
  return 0;
}


TIMER_HANDLE switch1TimeOutHandle= TIMER_NONE;

void switch1TimeOutAlarm(CO_Data * d, UNS32 unused) {
  switch1TimeOutHandle= TIMER_NONE;
  Switch1= 0;
  SWITCH1_OFF;
}

// TODO: differentiate between read and write!
UNS32 writeSwitchCallback(CO_Data* d, const indextable *, UNS8 bSubindex) {
  if((!Switch1) != (!SWITCH1_STATE)) {
    if(Switch1) {
      if(Switch1_TimeOut) {
        switch1TimeOutHandle= SetAlarm(d, 0, &switch1TimeOutAlarm, MS_TO_TIMEVAL(Switch1_TimeOut), 0);
        SWITCH1_ON;
        Switch1= 1;
      }
    } else {
      switch1TimeOutHandle= DelAlarm(switch1TimeOutHandle);
      SWITCH1_OFF;
    }
  }
  return 0;
}

//UNS32 writeSwitchCallback(CO_Data* d, const indextable *, UNS8 bSubindex) {
//  if((!Switch1) != (!SWITCH1_STATE)) {
//    if(Switch1) {
//      SWITCH1_ON;
//      Switch1= 1;
//    } else {
//      SWITCH1_OFF;
//    }
//  }
//  return 0;
//}
//
void setup() {
  LED_GREEN_INIT;
  LED_RED_INIT;
  SWITCH1_INIT;
  
  LED_RED_ON;
  LED_GREEN_ON;

  MsTimer2::set(62, blink); // 500ms period
  MsTimer2::start();
  
//  Serial.begin(9600);
  
  while(CAN.begin(CAN_500KBPS)!=0) delay(1000);
    
  // add keep trying and EMCY on success  
  CAN.init_Mask(0,0, 0x00FF);
  CAN.init_Filt(0,0, getNodeId(&ObjDict_Data)); // RxPDO and SDO
  CAN.init_Filt(1,0, getNodeId(&ObjDict_Data)); // same

  CAN.init_Mask(1,0, 0x07FF);
  CAN.init_Filt(2,0, 0x0000); // NMT
  CAN.init_Filt(3,0, 0x0080); // sync
  CAN.init_Filt(4,0, 0x0100); // time stamp
  CAN.init_Filt(5,0, 0x0000); // dummy
  
  initTimer();                                 	// Start timer for the CANopen stack
  setState(&ObjDict_Data, Initialisation);	// Init the state

  if(!(bmpOK= bmp.begin())) {
    EMCY_setError(&ObjDict_Data, CANopenDevErr_BMPErrorCode, CANopenErrReg_Manufacturer, 0);    
  }
  
  analogReference(INTERNAL);
  
  RegisterSetODentryCallBack(&ObjDict_Data, 0x2000, 0, readDHTCallback);
  RegisterSetODentryCallBack(&ObjDict_Data, 0x2002, 0, readBMPPressureCallback);
  RegisterSetODentryCallBack(&ObjDict_Data, 0x2003, 0, readBMPTemperatureCallback);
  RegisterSetODentryCallBack(&ObjDict_Data, 0x2004, 0, readCounterCallback);
  RegisterSetODentryCallBack(&ObjDict_Data, 0x2006, 0, rwLockCallback);
  RegisterSetODentryCallBack(&ObjDict_Data, 0x2007, 0, writeSwitchCallback);
  
//  Serial.print("gl: ");
//  Serial.println(glitchDetector, BIN);
  
  if (resetFlags & _BV(WDRF)) {
//    ApplicationMonitor.Dump(Serial);
    Watchdog::CCrashReport report;
    if(ApplicationMonitor.LoadLastReport(report)) {
//        Serial.println("WDT");
        EMCY_setError(&ObjDict_Data, CANopenSoftErr_WatchDog, CANopenErrReg_Manufacturer, ((uint16_t *)report.m_auAddress)[0]);
        EMCY_errorRecovered(&ObjDict_Data, CANopenSoftErr_WatchDog);
    } else {
        EMCY_setError(&ObjDict_Data, CANopenMonErr_UserRest, CANopenErrReg_Manufacturer, 0);
        EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_UserRest);
//        Serial.println("Ext");
    }
  } else {
      if (resetFlags & _BV(PORF))
//        Serial.println("PWR");
        ;
      else if (resetFlags & _BV(BORF)) {
        EMCY_setError(&ObjDict_Data, CANopenSoftErr_BrownOut, CANopenErrReg_Voltage, 0);
        EMCY_errorRecovered(&ObjDict_Data, CANopenSoftErr_BrownOut);
//        Serial.println("BOD");
      }
  }    
  ApplicationMonitor.EnableWatchdog(Watchdog::CApplicationMonitor::Timeout_2s);

//  if(glitchDetector==0x5555) {
//    EMCY_setError(&ObjDict_Data, CANopenGlitchErrorCode, CANOPEN_ERROR_REG_MANUFACTURER, 0);
//    EMCY_errorRecovered(&ObjDict_Data, CANopenGlitchErrorCode);
//  }
//  glitchDetector= 0x5555;
//  wdt_enable(WDTO_2S);
  LED_RED_OFF;
  LED_GREEN_OFF;
}

void loop() {
  if(coreTimerTrigger) {
    coreTimerTrigger= 0;
    TimeDispatch();                               // Call the time handler of the stack to adapt the elapsed time
  }

  // a message was received pass it to the CANstack
  if (canReceive(&m))	{		// a message reveived
    if(rx_error_state==rx_no_error)
      GreenBlink.setPattern(BlinkPattern::Flash1);
//    Serial.print("rcv");
//    Serial.println(m.cob_id, HEX);
    canDispatch(&ObjDict_Data, &m);         // process it
  }
  
  
  uint8_t ef= CAN.errorFlag();
  if(ef & MCP_CAN::EFlg_TxWar)
    setTxErrorState(tx_warning);
    
  if(ef & MCP_CAN::EFlg_TxBusOff) {
    setTxErrorState(tx_bus_off);
  } else if(ef & MCP_CAN::EFlg_TxEP)
    setTxErrorState(tx_passive);
  else if(tx_error_state!=tx_no_error) {
    if(CAN.checkTransmit())
      resetTxErrorState();
  }
  
  Main_Voltage= analogRead(A3) * VoltageScale;
  if(Main_Voltage<11.0) {
    underVoltageState= true;
    EMCY_setError(&ObjDict_Data, CANopenMonErr_UnderVoltage, CANopenErrReg_Voltage, 0);
  } else if(underVoltageState && Main_Voltage>11.7) {
    underVoltageState= false;
    EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_UnderVoltage);
  }    
  
  
  if(tx_last_error!=tx_error_state) {
    if(tx_error_state==tx_no_error) {
      EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_TxBusOff);
      EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_TxErr);
      EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_TxPassive);
      EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_TxAllBusy);
      EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_TxWarn);
    } else {
      switch(tx_error_state) {
        case tx_warning:
          EMCY_setError(&ObjDict_Data, CANopenMonErr_TxWarn, CANopenErrReg_Communication, 0);
          break;
        case tx_all_busy: // TODO: will probably not get sent !!
          EMCY_setError(&ObjDict_Data, CANopenMonErr_TxAllBusy, CANopenErrReg_Communication, 0);
          break;
        case tx_passive:
          EMCY_setError(&ObjDict_Data, CANopenMonErr_TxPassive, CANopenErrReg_Communication, 0);
          break;
        case tx_error: // TODO: will probably not get sent !!
          EMCY_setError(&ObjDict_Data, CANopenMonErr_TxErr, CANopenErrReg_Communication, 0);
          break;
        case tx_bus_off: // TODO: will probably not get sent !!
          EMCY_setError(&ObjDict_Data, CANopenMonErr_TxBusOff, CANopenErrReg_Communication, 0);
          break;
      }
      switch(tx_error_state) {
        case tx_bus_off:
          EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_TxErr);
        case tx_error:
          EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_TxPassive);
        case tx_passive:
          EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_TxAllBusy);
        case tx_all_busy:
          EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_TxWarn);
      }
    }
    tx_last_error= tx_error_state;
  }
  


  if(ef && MCP_CAN::EFlg_RxWar)
    setRxErrorState(rx_warning);
    
  if(ef & (MCP_CAN::EFlg_Rx1Ovr | MCP_CAN::EFlg_Rx0Ovr))
    setRxErrorState(rx_overflow);
  else if(ef && MCP_CAN::EFlg_RxEP)
    setRxErrorState(rx_passive);
  else if(GreenBlink.patternStarted())
    resetRxErrorState();
  
  if(rx_last_error!=rx_error_state) {
//    Serial.print("ES: ");
//    Serial.println(rx_error_state);
    if(rx_error_state==rx_no_error) {
      EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_RxOverFlow);
      EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_RxPassive);
      EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_RxWarn);
    } else {
      switch(rx_error_state) {
        case rx_warning:
          EMCY_setError(&ObjDict_Data, CANopenMonErr_RxWarn, CANopenErrReg_Communication, 0);
          break;
        case rx_passive:
          EMCY_setError(&ObjDict_Data, CANopenMonErr_RxPassive, CANopenErrReg_Communication, 0);
          break;
        case rx_overflow:
          EMCY_setError(&ObjDict_Data, CANopenMonErr_RxOverFlow, CANopenErrReg_Communication, 0);
          break;
      }
      switch(rx_error_state) {
        case rx_overflow:
          EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_RxPassive);
        case rx_passive:
          EMCY_errorRecovered(&ObjDict_Data, CANopenMonErr_RxWarn);
      }
    }
    rx_last_error= rx_error_state;
  }

  ApplicationMonitor.IAmAlive();
}
