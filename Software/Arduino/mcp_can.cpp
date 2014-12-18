/*
  mcp_can.cpp
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.

  Author:Loovee
  Contributor: Cory J. Fowler
  2014-1-16
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
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/
#include "mcp_can.h"
#include "timer.h"
#include <util/delay.h> 
#include <avr/io.h>


#define spi_read() spi_readwrite(0x00)
//#define SPICS 10
#define DDR_SPI DDRB
#define DD_MOSI DDB3
#define DD_MISS DDB4
#define DD_SCK DDB5
#define DD_SS DDB2
#define DD_CS DDD7

#define DDR_CS DDRD
#define PORT_CS PORTD
#define P_CS PD7

#define MCP2515_SELECT()   PORT_CS&= ~_BV(P_CS)
#define MCP2515_UNSELECT() PORT_CS|= _BV(P_CS)

typedef uint8_t byte;

INT8U   m_nExtFlg;                                                  /* identifier xxxID             */
								    /* either extended (the 29 LSB) */
								    /* or standard (the 11 LSB)     */
INT32U  m_nID;                                                      /* can id                       */
INT8U   m_nDlc;                                                     /* data length:                 */
INT8U   m_nDta[MAX_CHAR_IN_MESSAGE];                            	/* data                         */
INT8U   m_nRtr;                                                     /* rtr                          */
INT8U   m_nfilhit;


INT8U begin(INT8U speedset);
INT8U sendMsgBufRTR(INT32U id, INT8U ext, INT8U rtr, INT8U len, INT8U *buf);
INT8U checkReceive(void);
INT8U readMsgBufID(INT32U *ID, INT8U *len, INT8U buf[]);
INT8U readMsgBuf(INT8U *len, INT8U buf[]);
INT32U getCanId(void);

unsigned char canInit(unsigned int bitrate) {
  if(bitrate==500)
    return begin(CAN_500KBPS);
  else
    return CAN_FAILINIT;
}

unsigned char canSend(CAN_PORT notused, Message *m) {
  return sendMsgBufRTR(m->cob_id, 0, m->rtr, m->len, m->data);
}


unsigned char canReceive(Message *m) {
    if(CAN_MSGAVAIL == checkReceive())            // check if data coming
    {
      //      readMsgBufID(&(m->cob_id), &(m->len), m->data);
      readMsgBuf(&(m->len), m->data);
      m->cob_id= getCanId();
      m->rtr= m_nRtr;
      return 1;
    } else
      return 0;
}


unsigned char canChangeBaudRate_driver( CAN_HANDLE fd, char* baud) {
  return 1;
}

byte spi_readwrite(byte _data) {
  SPDR = _data;
  while (!(SPSR & _BV(SPIF)))
    ;
  return SPDR;
}


/*********************************************************************************************************
** Function name:           mcp2515_reset
** Descriptions:            reset the device
*********************************************************************************************************/
void mcp2515_reset(void)
{
    MCP2515_SELECT();
    spi_readwrite(MCP_RESET);
    MCP2515_UNSELECT();
    _delay_ms(10);
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegister
** Descriptions:            read register
*********************************************************************************************************/
INT8U mcp2515_readRegister(const INT8U address)                                                                     
{
    INT8U ret;

    MCP2515_SELECT();
    spi_readwrite(MCP_READ);
    spi_readwrite(address);
    ret = spi_read();
    MCP2515_UNSELECT();

    return ret;
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegisterS
** Descriptions:            read registerS
*********************************************************************************************************/
void mcp2515_readRegisterS(const INT8U address, INT8U values[], const INT8U n)
{
	INT8U i;
	MCP2515_SELECT();
	spi_readwrite(MCP_READ);
	spi_readwrite(address);
	// mcp2515 has auto-increment of address-pointer
	for (i=0; i<n && i<CAN_MAX_CHAR_IN_MESSAGE; i++) {
		values[i] = spi_read();
	}
	MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegister
** Descriptions:            set register
*********************************************************************************************************/
void mcp2515_setRegister(const INT8U address, const INT8U value)
{
    MCP2515_SELECT();
    spi_readwrite(MCP_WRITE);
    spi_readwrite(address);
    spi_readwrite(value);
    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegisterS
** Descriptions:            set registerS
*********************************************************************************************************/
void mcp2515_setRegisterS(const INT8U address, const INT8U values[], const INT8U n)
{
    INT8U i;
    MCP2515_SELECT();
    spi_readwrite(MCP_WRITE);
    spi_readwrite(address);
       
    for (i=0; i<n; i++) 
    {
        spi_readwrite(values[i]);
    }
    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           mcp2515_modifyRegister
** Descriptions:            set bit of one register
*********************************************************************************************************/
void mcp2515_modifyRegister(const INT8U address, const INT8U mask, const INT8U data)
{
    MCP2515_SELECT();
    spi_readwrite(MCP_BITMOD);
    spi_readwrite(address);
    spi_readwrite(mask);
    spi_readwrite(data);
    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           mcp2515_readStatus
** Descriptions:            read mcp2515's Status
*********************************************************************************************************/
INT8U mcp2515_readStatus(void)                             
{
	INT8U i;
	MCP2515_SELECT();
	spi_readwrite(MCP_READ_STATUS);
	i = spi_read();
	MCP2515_UNSELECT();
	
	return i;
}

/*********************************************************************************************************
** Function name:           mcp2515_setCANCTRL_Mode
** Descriptions:            set control mode
*********************************************************************************************************/
INT8U mcp2515_setCANCTRL_Mode(const INT8U newmode)
{
    INT8U i;

    mcp2515_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode);

    i = mcp2515_readRegister(MCP_CANSTAT);
    i &= MODE_MASK;

    if ( i == newmode ) 
    {
        return MCP2515_OK;
    }

    return MCP2515_FAIL;

}

/*********************************************************************************************************
** Function name:           mcp2515_configRate
** Descriptions:            set boadrate
*********************************************************************************************************/
INT8U mcp2515_configRate(const INT8U canSpeed)            
{
    INT8U set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canSpeed) 
    {
        case (CAN_5KBPS):
        cfg1 = MCP_16MHz_5kBPS_CFG1;
        cfg2 = MCP_16MHz_5kBPS_CFG2;
        cfg3 = MCP_16MHz_5kBPS_CFG3;
        break;

        case (CAN_10KBPS):
        cfg1 = MCP_16MHz_10kBPS_CFG1;
        cfg2 = MCP_16MHz_10kBPS_CFG2;
        cfg3 = MCP_16MHz_10kBPS_CFG3;
        break;

        case (CAN_20KBPS):
        cfg1 = MCP_16MHz_20kBPS_CFG1;
        cfg2 = MCP_16MHz_20kBPS_CFG2;
        cfg3 = MCP_16MHz_20kBPS_CFG3;
        break;
        
        case (CAN_31K25BPS):
        cfg1 = MCP_16MHz_31k25BPS_CFG1;
        cfg2 = MCP_16MHz_31k25BPS_CFG2;
        cfg3 = MCP_16MHz_31k25BPS_CFG3;
        break;

        case (CAN_40KBPS):
        cfg1 = MCP_16MHz_40kBPS_CFG1;
        cfg2 = MCP_16MHz_40kBPS_CFG2;
        cfg3 = MCP_16MHz_40kBPS_CFG3;
        break;

        case (CAN_50KBPS):
        cfg1 = MCP_16MHz_50kBPS_CFG1;
        cfg2 = MCP_16MHz_50kBPS_CFG2;
        cfg3 = MCP_16MHz_50kBPS_CFG3;
        break;

        case (CAN_80KBPS):
        cfg1 = MCP_16MHz_80kBPS_CFG1;
        cfg2 = MCP_16MHz_80kBPS_CFG2;
        cfg3 = MCP_16MHz_80kBPS_CFG3;
        break;

        case (CAN_100KBPS):                                             /* 100KBPS                  */
        cfg1 = MCP_16MHz_100kBPS_CFG1;
        cfg2 = MCP_16MHz_100kBPS_CFG2;
        cfg3 = MCP_16MHz_100kBPS_CFG3;
        break;

        case (CAN_125KBPS):
        cfg1 = MCP_16MHz_125kBPS_CFG1;
        cfg2 = MCP_16MHz_125kBPS_CFG2;
        cfg3 = MCP_16MHz_125kBPS_CFG3;
        break;

        case (CAN_200KBPS):
        cfg1 = MCP_16MHz_200kBPS_CFG1;
        cfg2 = MCP_16MHz_200kBPS_CFG2;
        cfg3 = MCP_16MHz_200kBPS_CFG3;
        break;

        case (CAN_250KBPS):
        cfg1 = MCP_16MHz_250kBPS_CFG1;
        cfg2 = MCP_16MHz_250kBPS_CFG2;
        cfg3 = MCP_16MHz_250kBPS_CFG3;
        break;

        case (CAN_500KBPS):
        cfg1 = MCP_16MHz_500kBPS_CFG1;
        cfg2 = MCP_16MHz_500kBPS_CFG2;
        cfg3 = MCP_16MHz_500kBPS_CFG3;
        break;
        
        case (CAN_1000KBPS):
        cfg1 = MCP_16MHz_1000kBPS_CFG1;
        cfg2 = MCP_16MHz_1000kBPS_CFG2;
        cfg3 = MCP_16MHz_1000kBPS_CFG3;
        break;  

        default:
        set = 0;
        break;
    }

    if (set) {
        mcp2515_setRegister(MCP_CNF1, cfg1);
        mcp2515_setRegister(MCP_CNF2, cfg2);
        mcp2515_setRegister(MCP_CNF3, cfg3);
        return MCP2515_OK;
    }
    else {
        return MCP2515_FAIL;
    }
}

/*********************************************************************************************************
** Function name:           mcp2515_initCANBuffers
** Descriptions:            init canbuffers
*********************************************************************************************************/
void mcp2515_initCANBuffers(void)
{
    INT8U i, a1, a2, a3;
    
    //INT8U std = 0;               
    //INT8U ext = 1;
    //INT32U ulMask = 0x00, ulFilt = 0x00;


    //mcp2515_write_id(MCP_RXM0SIDH, ext, ulMask);			/*Set both masks to 0           */
    //mcp2515_write_id(MCP_RXM1SIDH, ext, ulMask);			/*Mask register ignores ext bit */
    
                                                            /* Set all filters to 0         */
    //mcp2515_write_id(MCP_RXF0SIDH, ext, ulFilt);			/* RXB0: extended               */
    //mcp2515_write_id(MCP_RXF1SIDH, std, ulFilt);			/* RXB1: standard               */
    //mcp2515_write_id(MCP_RXF2SIDH, ext, ulFilt);			/* RXB2: extended               */
    //mcp2515_write_id(MCP_RXF3SIDH, std, ulFilt);			/* RXB3: standard               */
    //mcp2515_write_id(MCP_RXF4SIDH, ext, ulFilt);
    //mcp2515_write_id(MCP_RXF5SIDH, std, ulFilt);

                                                                        /* Clear, deactivate the three  */
                                                                        /* transmit buffers             */
                                                                        /* TXBnCTRL -> TXBnD7           */
    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++) {                                          /* in-buffer loop               */
        mcp2515_setRegister(a1, 0);
        mcp2515_setRegister(a2, 0);
        mcp2515_setRegister(a3, 0);
        a1++;
        a2++;
        a3++;
    }
    mcp2515_setRegister(MCP_RXB0CTRL, 0);
    mcp2515_setRegister(MCP_RXB1CTRL, 0);
}

/*********************************************************************************************************
** Function name:           mcp2515_init
** Descriptions:            init the device
*********************************************************************************************************/
INT8U mcp2515_init(const INT8U canSpeed)                       /* mcp2515init                  */
{

  INT8U res;

    mcp2515_reset();

    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
    {
      _delay_ms(10);
      return res;
    }
    _delay_ms(10);

                                                                        /* set boadrate                 */
    if(mcp2515_configRate(canSpeed))
    {
      _delay_ms(10);
      return 8;
    }
    _delay_ms(10);

    if ( res == MCP2515_OK ) {

                                                                        /* init canbuffers              */
        mcp2515_initCANBuffers();

                                                                        /* interrupt mode               */
        mcp2515_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);

#if (DEBUG_RXANY==1)
                                                                        /* enable both receive-buffers  */
                                                                        /* to receive any message       */
                                                                        /* and enable rollover          */
        mcp2515_modifyRegister(MCP_RXB0CTRL,
        MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
        MCP_RXB_RX_ANY | MCP_RXB_BUKT_MASK);
        mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
        MCP_RXB_RX_ANY);
#else
                                                                        /* enable both receive-buffers  */
                                                                        /* to receive messages          */
                                                                        /* with std. and ext. identifie */
                                                                        /* rs                           */
                                                                        /* and enable rollover          */
        mcp2515_modifyRegister(MCP_RXB0CTRL,
        MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
        MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK );
        mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
        MCP_RXB_RX_STDEXT);
#endif
                                                                        /* enter normal mode            */
        res = mcp2515_setCANCTRL_Mode(MODE_NORMAL);                                                                
        if(res)
        {
//         Serial.print("Enter Normal Mode Fall!!\r\n");
          return res;
        }


            _delay_ms(10);

    }
    return res;

}

/*********************************************************************************************************
** Function name:           mcp2515_write_id
** Descriptions:            write can id
*********************************************************************************************************/
void mcp2515_write_id( const INT8U mcp_addr, const INT8U ext, const INT32U id )
{
    uint16_t canid;
    INT8U tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if ( ext == 1) 
    {
        tbufdata[MCP_EID0] = (INT8U) (canid & 0xFF);
        tbufdata[MCP_EID8] = (INT8U) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (INT8U) (canid & 0x03);
        tbufdata[MCP_SIDL] += (INT8U) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (INT8U) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_SIDH] = (INT8U) (canid >> 3 );
        tbufdata[MCP_SIDL] = (INT8U) ((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    mcp2515_setRegisterS( mcp_addr, tbufdata, 4 );
}

/*********************************************************************************************************
** Function name:           mcp2515_read_id
** Descriptions:            read can id
*********************************************************************************************************/
void mcp2515_read_id( const INT8U mcp_addr, INT8U* ext, INT32U* id )
{
    INT8U tbufdata[4];

    *ext = 0;
    *id = 0;

    mcp2515_readRegisterS( mcp_addr, tbufdata, 4 );

    *id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M ) 
    {
                                                                        /* extended id                  */
        *id = (*id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id<<8) + tbufdata[MCP_EID8];
        *id = (*id<<8) + tbufdata[MCP_EID0];
        *ext = 1;
    }
}

/*********************************************************************************************************
** Function name:           mcp2515_write_canMsg
** Descriptions:            write msg
*********************************************************************************************************/
void mcp2515_write_canMsg( const INT8U buffer_sidh_addr)
{
    INT8U mcp_addr;
    mcp_addr = buffer_sidh_addr;
    mcp2515_setRegisterS(mcp_addr+5, m_nDta, m_nDlc );                  /* write data bytes             */
    if ( m_nRtr == 1)                                                   /* if RTR set bit in byte       */
    {
        m_nDlc |= MCP_RTR_MASK;  
    }
    mcp2515_setRegister((mcp_addr+4), m_nDlc );                        /* write the RTR and DLC        */
    mcp2515_write_id(mcp_addr, m_nExtFlg, m_nID );                     /* write CAN id                 */

}

/*********************************************************************************************************
** Function name:           mcp2515_read_canMsg
** Descriptions:            read message
*********************************************************************************************************/
void mcp2515_read_canMsg( const INT8U buffer_sidh_addr)        /* read can msg                 */
{
    INT8U mcp_addr, ctrl;

    mcp_addr = buffer_sidh_addr;

    mcp2515_read_id( mcp_addr, &m_nExtFlg,&m_nID );

    ctrl = mcp2515_readRegister( mcp_addr-1 );
    m_nDlc = mcp2515_readRegister( mcp_addr+4 );

    if ((ctrl & 0x08)) {
        m_nRtr = 1;
    }
    else {
        m_nRtr = 0;
    }

    m_nDlc &= MCP_DLC_MASK;
    mcp2515_readRegisterS( mcp_addr+5, &(m_nDta[0]), m_nDlc );
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
*********************************************************************************************************/
void mcp2515_start_transmit(const INT8U mcp_addr)              /* start transmit               */
{
    mcp2515_modifyRegister( mcp_addr-1 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
*********************************************************************************************************/
INT8U mcp2515_getNextFreeTXBuf(INT8U *txbuf_n)                 /* get Next free txbuf          */
{
    INT8U res, i, ctrlval;
    INT8U ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

    res = MCP_ALLTXBUSY;
    *txbuf_n = 0x00;

                                                                        /* check all 3 TX-Buffers       */
    for (i=0; i<MCP_N_TXBUFFERS; i++) {
        ctrlval = mcp2515_readRegister( ctrlregs[i] );
        if ( (ctrlval & MCP_TXB_TXREQ_M) == 0 ) {
            *txbuf_n = ctrlregs[i]+1;                                   /* return SIDH-address of Buffe */
                                                                        /* r                            */
            res = MCP2515_OK;
            return res;                                                 /* ! function exit              */
        }
    }
    return res;
}

/*********************************************************************************************************
** Function name:           init
** Descriptions:            init can and set speed
*********************************************************************************************************/
INT8U begin(INT8U speedset)
{
    INT8U res;

  // When the SS pin is set as OUTPUT, it can be used as
  // a general purpose output port (it doesn't influence
  // SPI operations).
  // Warning: if the SS pin ever becomes a LOW INPUT then SPI
  // automatically switches to Slave, so the data direction of
  // the SS pin MUST be kept as OUTPUT.
    DDR_SPI|= _BV(DD_SS);
    DDR_CS|= _BV(DD_CS);
    MCP2515_UNSELECT();

    SPCR |= _BV(MSTR);
    SPCR |= _BV(SPE);

    // Set direction register for SCK and MOSI pin.
    // MISO pin automatically overrides to INPUT.
    // By doing this AFTER enabling SPI, we avoid accidentally
    // clocking in a single bit since the lines go directly
    // from "input" to SPI control.  
    // http://code.google.com/p/arduino/issues/detail?id=888
    DDR_SPI|= _BV(DD_MOSI);
    DDR_SPI|= _BV(DD_SCK);
    
    res = mcp2515_init(speedset);
    if (res == MCP2515_OK) return CAN_OK;
    else return CAN_FAILINIT;
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            init canid Masks
*********************************************************************************************************/
INT8U init_Mask(INT8U num, INT8U ext, INT32U ulData)
{
    INT8U res = MCP2515_OK;
    _delay_ms(10);

    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0){
      _delay_ms(10);
      return res;
    }
    
    if (num == 0){
        mcp2515_write_id(MCP_RXM0SIDH, ext, ulData);
    }
    else if(num == 1){
        mcp2515_write_id(MCP_RXM1SIDH, ext, ulData);
    }
    else res =  MCP2515_FAIL;
    
    res = mcp2515_setCANCTRL_Mode(MODE_NORMAL);
    if(res > 0){
      _delay_ms(10);
      return res;
    }
    _delay_ms(10);
    return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            init canid filters
*********************************************************************************************************/
INT8U init_Filt(INT8U num, INT8U ext, INT32U ulData)
{
    INT8U res = MCP2515_OK;
    _delay_ms(10);
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
    {
      _delay_ms(10);
      return res;
    }
    
    switch( num )
    {
        case 0:
        mcp2515_write_id(MCP_RXF0SIDH, ext, ulData);
        break;

        case 1:
        mcp2515_write_id(MCP_RXF1SIDH, ext, ulData);
        break;

        case 2:
        mcp2515_write_id(MCP_RXF2SIDH, ext, ulData);
        break;

        case 3:
        mcp2515_write_id(MCP_RXF3SIDH, ext, ulData);
        break;

        case 4:
        mcp2515_write_id(MCP_RXF4SIDH, ext, ulData);
        break;

        case 5:
        mcp2515_write_id(MCP_RXF5SIDH, ext, ulData);
        break;

        default:
        res = MCP2515_FAIL;
    }
    
    res = mcp2515_setCANCTRL_Mode(MODE_NORMAL);
    if(res > 0)
    {
      _delay_ms(10);
      return res;
    }
    _delay_ms(10);
    
    return res;
}

/*********************************************************************************************************
** Function name:           setMsg
** Descriptions:            set can message, such as dlc, id, dta[] and so on
*********************************************************************************************************/
INT8U setMsgRTR(INT32U id, INT8U ext, INT8U len, INT8U rtr, INT8U *pData)
{
    int i = 0;
    m_nExtFlg = ext;
    m_nID     = id;
    m_nDlc    = len;
    m_nRtr    = rtr;
    for(i = 0; i<MAX_CHAR_IN_MESSAGE; i++)
    {
        m_nDta[i] = *(pData+i);
    }
    return MCP2515_OK;
}


/*********************************************************************************************************
** Function name:           setMsg
** Descriptions:            set can message, such as dlc, id, dta[] and so on
*********************************************************************************************************/
INT8U setMsg(INT32U id, INT8U ext, INT8U len, INT8U *pData)
{
    int i = 0;
    m_nExtFlg = ext;
    m_nID     = id;
    m_nDlc    = len;
    for(i = 0; i<MAX_CHAR_IN_MESSAGE; i++)
    {
        m_nDta[i] = *(pData+i);
    }
    return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           clearMsg
** Descriptions:            set all message to zero
*********************************************************************************************************/
INT8U clearMsg()
{
    m_nID       = 0;
    m_nDlc      = 0;
    m_nExtFlg   = 0;
    m_nRtr      = 0;
    m_nfilhit   = 0;
    for(int i = 0; i<m_nDlc; i++ )
      m_nDta[i] = 0x00;

    return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
*********************************************************************************************************/
INT8U sendMsg()
{
    INT8U res, res1, txbuf_n;
    uint16_t uiTimeOut = 0;

    do {
        res = mcp2515_getNextFreeTXBuf(&txbuf_n);                       /* info = addr.                 */
        uiTimeOut++;
    } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

    if(uiTimeOut == TIMEOUTVALUE) 
    {   
        return CAN_GETTXBFTIMEOUT;                                      /* get tx buff time out         */
    }
    uiTimeOut = 0;
    mcp2515_write_canMsg( txbuf_n);
    mcp2515_start_transmit( txbuf_n );
    do
    {
        uiTimeOut++;        
        res1= mcp2515_readRegister(txbuf_n-1);  			                /* read send buff ctrl reg 	*/
        res1 = res1 & 0x08;                               		
    }while(res1 && (uiTimeOut < TIMEOUTVALUE));   
    if(uiTimeOut == TIMEOUTVALUE)                                       /* send msg timeout             */	
    {
        return CAN_SENDMSGTIMEOUT;
    }
    return CAN_OK;

}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            send buf
*********************************************************************************************************/
INT8U sendMsgBufRTR(INT32U id, INT8U ext, INT8U rtr, INT8U len, INT8U *buf)
{
    setMsgRTR(id, ext, len, rtr, buf);
    sendMsg();
    return 1;
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            send buf
*********************************************************************************************************/
INT8U sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf)
{
    setMsg(id, ext, len, buf);
    sendMsg();
    return 1;
}


/*********************************************************************************************************
** Function name:           readMsg
** Descriptions:            read message
*********************************************************************************************************/
INT8U readMsg()
{
    INT8U stat, res;

    stat = mcp2515_readStatus();

    if ( stat & MCP_STAT_RX0IF )                                        /* Msg in Buffer 0              */
    {
        mcp2515_read_canMsg( MCP_RXBUF_0);
        mcp2515_modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
        res = CAN_OK;
    }
    else if ( stat & MCP_STAT_RX1IF )                                   /* Msg in Buffer 1              */
    {
        mcp2515_read_canMsg( MCP_RXBUF_1);
        mcp2515_modifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
        res = CAN_OK;
    }
    else 
    {
        res = CAN_NOMSG;
    }
    return res;
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            read message buf
*********************************************************************************************************/
INT8U readMsgBuf(INT8U *len, INT8U buf[])
{
    INT8U  rc;
    
    rc = readMsg();
    
    if (rc == CAN_OK) {
       *len = m_nDlc;
       for(int i = 0; i<m_nDlc; i++) {
         buf[i] = m_nDta[i];
       } 
    } else {
       	 *len = 0;
    }
    return rc;
}

/*********************************************************************************************************
** Function name:           readMsgBufID
** Descriptions:            read message buf and can bus source ID
*********************************************************************************************************/
INT8U readMsgBufID(INT32U *ID, INT8U *len, INT8U buf[])
{
    INT8U rc;
    rc = readMsg();

    if (rc == CAN_OK) {
       *len = m_nDlc;
       *ID  = m_nID;
       for(int i = 0; i<m_nDlc && i < MAX_CHAR_IN_MESSAGE; i++) {
          buf[i] = m_nDta[i];
       }
    } else {
       *len = 0;
    }
    return rc;
}

/*********************************************************************************************************
** Function name:           checkReceive
** Descriptions:            check if got something
*********************************************************************************************************/
INT8U checkReceive(void)
{
    INT8U res;
    res = mcp2515_readStatus();                                         /* RXnIF in Bit 1 and 0         */
    if ( res & MCP_STAT_RXIF_MASK ) 
    {
        return CAN_MSGAVAIL;
    }
    else 
    {
        return CAN_NOMSG;
    }
}

/*********************************************************************************************************
** Function name:           checkError
** Descriptions:            if something error
*********************************************************************************************************/
INT8U checkError(void)
{
    INT8U eflg = mcp2515_readRegister(MCP_EFLG);

    if ( eflg & MCP_EFLG_ERRORMASK ) 
    {
        return CAN_CTRLERROR;
    }
    else 
    {
        return CAN_OK;
    }
}

/*********************************************************************************************************
** Function name:           getCanId
** Descriptions:            when receive something ,u can get the can id!!
*********************************************************************************************************/
INT32U getCanId(void)
{
    return m_nID;
} 

/*********************************************************************************************************
** Function name:           isRemoteRequest
** Descriptions:            when receive something ,u can check if it was a request
*********************************************************************************************************/
INT8U isRemoteRequest(void)
{
    return m_nRtr;
} 
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
