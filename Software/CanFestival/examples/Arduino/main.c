/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN
AVR Port: Andreas GLAUSER and Peter CHRISTEN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
/******************************************************************************
Project description:
Test projekt for a DS 401 slave, running on Atmel's STK500 with AT90CAN128
Short description:
  PORTA:	Inputs (Keys, low active)
  PORTB:	Outputs (LEDs, low active)
  PORTC:	Node ID (1 BCD switch, low active)

******************************************************************************/

#include <avr/io.h>
#include "hardware.h"
#include "canfestival.h"
#include "mcp_can.h"
#include "ObjDict.h"
#include "ds401.h"
#include "timerscfg.h"

volatile unsigned char timer_interrupt = 0;		// Set if timer interrupt elapsed
unsigned char inputs;

// CAN
unsigned char nodeID;
unsigned char digital_input[1] = {0};
unsigned char digital_output[1] = {0};

static Message m = Message_Initializer;		// contain a CAN message

void sys_init();

// macros to handle the schedule timer
#define sys_timer			timer_interrupt
#define reset_sys_timer()		timer_interrupt = 0
#define CYCLE_TIME	        	1000     	// Sample Timebase [us]

int main(void)
{
  sys_init();                                   // Initialize system
  canInit(CAN_BAUDRATE);         		// Initialize the CANopen bus
  initTimer();                                 	// Start timer for the CANopen stack
  nodeID = 23;				// Read node ID first
  setNodeId (&ObjDict_Data, nodeID);
  setState(&ObjDict_Data, Initialisation);	// Init the state

  for(;;)		                        // forever loop
  {
    if(coreTimerTrigger) {
      coreTimerTrigger= 0;
      TimeDispatch();                               // Call the time handler of the stack to adapt the elapsed time
    }
    if (sys_timer)	                        // Cycle timer, invoke action on every time slice
    {
      reset_sys_timer();	                // Reset timer
      digital_input[0] = get_inputs();
      digital_input_handler(&ObjDict_Data, digital_input, sizeof(digital_input));
      digital_output_handler(&ObjDict_Data, digital_output, sizeof(digital_output));
      set_outputs(digital_output[0]);
    }
    // a message was received pass it to the CANstack
    if (canReceive(&m))	{		// a message reveived
      canDispatch(&ObjDict_Data, &m);         // process it
    } else
    {
      // Enter sleep mode
      #ifdef WD_SLEEP		// Watchdog and Sleep
      wdt_reset();
      sleep_enable();
      sleep_cpu();
      #endif				// Watchdog and Sleep
    }
  }
}

void sys_init()
/******************************************************************************
Initialize the relays, the main states and the modbus protocol stack.
INPUT	LOCK_STATES *lock_states
OUTPUT	void
******************************************************************************/
{
  PORTC= INPUT_MASK;	                        // Inputs (Keys, low active) with pullup
  DDRC= 0x00;		                // 
  
  PORTD|= OUTPUT_MASK;	                        // Outputs (LEDs, low active) all 1
  DDRD|= OUTPUT_MASK;		                // 

// Set timer 0 for main schedule time

  TCCR0A|= _BV(WGM01); 		// Timer 0 CTC
  TCCR0B|= _BV(CS01) | _BV(CS00);// Timer 0 mit CK/64 starten = 250000Hz == 0.004ms pro tick
  TIMSK0 = _BV(OCIE0A);		        // Timer Interrupts: Timer 0 Compare
  OCR0A = (unsigned char)(F_CPU / 64 * CYCLE_TIME/1000000 - 1);	// Reloadvalue for timer 0
  #ifdef WD_SLEEP		// Watchdog and Sleep
  wdt_reset();
  wdt_enable(WDTO_15MS);   	// Watchdogtimer start with 16 ms timeout
  #endif			// Watchdog and Sleep
  sei();         // Enable Interrupts
}


#ifdef  __IAR_SYSTEMS_ICC__
#pragma type_attribute = __interrupt
#pragma vector=TIMER0_COMP_vect
void TIMER0_OVF_interrupt(void)
#else	// GCC
ISR (TIMER0_COMPA_vect)
#endif	// GCC
/******************************************************************************
Interruptserviceroutine Timer 2 Compare A for the main cycle
******************************************************************************/

{
  timer_interrupt = 1;	// Set flag
}
