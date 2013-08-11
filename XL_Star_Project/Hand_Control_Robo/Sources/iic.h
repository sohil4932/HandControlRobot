/***********************************************************************************************\
* Freescale MMA8451,2,3Q Driver
*
* Filename: iic.h
*
*
* (c) Copyright 2010, Freescale, Inc.  All rights reserved.
*
* No part of this document must be reproduced in any form - including copied,
* transcribed, printed or by any electronic means - without specific written
* permission from Freescale Semiconductor.
*
\***********************************************************************************************/
#ifndef _IIC_H_
#define _IIC_H_

/***********************************************************************************************\
* Public macros
\***********************************************************************************************/

#ifdef _MC9S08QE64_H

/*** IIC1A - IIC Address Register; 0x00000030 ***/
#define IICA                            IIC1A

/*** IIC1F - IIC Frequency Divider Register; 0x00000031 ***/
#define IICF                            IIC1F

/*** IIC1C1 - IIC Control Register 1; 0x00000032 ***/
#define IICC1                           IIC1C1
#define IICC1_RSTA                      IIC1C1_RSTA
#define IICC1_TXAK                      IIC1C1_TXAK
#define IICC1_TX                        IIC1C1_TX
#define IICC1_MST                       IIC1C1_MST
#define IICC1_IICIE                     IIC1C1_IICIE
#define IICC1_IICEN                     IIC1C1_IICEN
    
#define IICC1_RSTA_MASK                 IIC1C1_RSTA_MASK
#define IICC1_TXAK_MASK                 IIC1C1_TXAK_MASK
#define IICC1_TX_MASK                   IIC1C1_TX_MASK
#define IICC1_MST_MASK                  IIC1C1_MST_MASK
#define IICC1_IICIE_MASK                IIC1C1_IICIE_MASK
#define IICC1_IICEN_MASK                IIC1C1_IICEN_MASK

#define IICC                            IIC1C
#define IICC_RSTA                       IIC1C_RSTA
#define IICC_TXAK                       IIC1C_TXAK
#define IICC_TX                         IIC1C_TX
#define IICC_MST                        IIC1C_MST
#define IICC_IICIE                      IIC1C_IICIE
#define IICC_IICEN                      IIC1C_IICEN
    
#define IICC_RSTA_MASK                  IIC1C_RSTA_MASK
#define IICC_TXAK_MASK                  IIC1C_TXAK_MASK
#define IICC_TX_MASK                    IIC1C_TX_MASK
#define IICC_MST_MASK                   IIC1C_MST_MASK
#define IICC_IICIE_MASK                 IIC1C_IICIE_MASK
#define IICC_IICEN_MASK                 IIC1C_IICEN_MASK
    
/*** IIC1S - IIC Status Register; 0x00000033 ***/
#define IICS                            IIC1S
#define IICS_RXAK                       IIC1S_RXAK
#define IICS_IICIF                      IIC1S_IICIF
#define IICS_SRW                        IIC1S_SRW
#define IICS_ARBL                       IIC1S_ARBL
#define IICS_BUSY                       IIC1S_BUSY
#define IICS_IAAS                       IIC1S_IAAS
#define IICS_TCF                        IIC1S_TCF

#define IICS_RXAK_MASK                  IIC1S_RXAK_MASK
#define IICS_IICIF_MASK                 IIC1S_IICIF_MASK
#define IICS_SRW_MASK                   IIC1S_SRW_MASK
#define IICS_ARBL_MASK                  IIC1S_ARBL_MASK
#define IICS_BUSY_MASK                  IIC1S_BUSY_MASK
#define IICS_IAAS_MASK                  IIC1S_IAAS_MASK
#define IICS_TCF_MASK                   IIC1S_TCF_MASK

/*** IIC1D - IIC Data I/O Register; 0x00000034 ***/
#define IICD                            IIC1D

/*** IIC1C2 - IIC Control Register 2; 0x00000035 ***/

#define IICC2                           IIC1C2
#define IICC2_AD8                       IIC1C2_AD8
#define IICC2_AD9                       IIC1C2_AD9
#define IICC2_AD10                      IIC1C2_AD10
#define IICC2_ADEXT                     IIC1C2_ADEXT
#define IICC2_GCAEN                     IIC1C2_GCAEN
#define IICC2_AD_8                      IIC1C2_AD_8
#define IICC2_AD                        IIC1C2_AD

#define IICC2_AD8_MASK                  IIC1C2_AD8_MASK
#define IICC2_AD9_MASK                  IIC1C2_AD9_MASK
#define IICC2_AD10_MASK                 IIC1C2_AD10_MASK
#define IICC2_ADEXT_MASK                IIC1C2_ADEXT_MASK
#define IICC2_GCAEN_MASK                IIC1C2_GCAEN_MASK
#define IICC2_AD_8_MASK                 IIC1C2_AD_8_MASK
#define IICC2_AD_8_BITNUM               IIC1C2_AD_8_BITNUM

#endif    /* _MC9S08QE64_H */


/***********************************************************************************************
**
**  Inter-Integrated Circuit (IIC)
**
**  0x0030  IICA      IIC Address Register
**  0x0031  IICF      IIC Frequency Divider Register
**  0x0032  IICC1     IIC Control Register 1
**  0x0033  IICS      IIC Status Register
**  0x0034  IICD      IIC Data I/O Register
**  0x0035  IICC2     IIC Control Register 2
**
**  IIC target bit rate = 400k
**  MCU bus frequency = 9.216MHz
**
**  IIC MULT = 0 ;  hence mul = 1
**  IIC ICR  = 2 ;  hence scl = 24
**
**  IIC bit rate = bus / (mul * scl)
**               = 9.216MHz / (1 * 24)
**               = 384kHz
**
**    In addtion, SDA Hold Value = 8
**                SCL Start Hold Value = 8
**                SDA Stop Hold Value = 13
**
**    Hence,
**      SDA hold time = (1/bus) * mul * SDA Hold Value
**                    = 109ns * 1 * 8
**                    = 868ns
**
**      SCL Start hold time = (1/bus) * mul * SCL Start Hold Value
**                          = 109ns * 1 * 8
**                          = 868ns
**
**      SDA Stop hold time  = (1/bus) * mul * SDA Stop Hold Value
**                          = 109ns * 1 * 13
**                          = 1411ns
*/

#define init_IICA     0x00

#define init_IICF     0b00000010
/*                      00000000 = reset
**                      ||||||||
**                      ||++++++-- ICR      =2  : scl = 24
**                      ++-------- MULT     =0  : mul = 1
*/

#define init_IICC1    0b10000000
/*                      00000000 = reset
**                      ||||||xx
**                      |||||+---- RSTA
**                      ||||+----- TXAK     =0  : send ACK
**                      |||+------ TX       =0  : Receive mode
**                      ||+------- MST      =0  : Initial state for Master mode
**                      |+-------- IICIE    =0  : IIC interrupts disabled
**                      +--------- IICEN    =1  : IIC module enabled
*/

#define init_IICC2    0b00000000
/*                      00000000 = reset
**                      ||xxx|||
**                      ||   +++-- AD10-8   =0  : upper 3 bits of 10-bit slave address
**                      |+-------- ADEXT    =0  : 7-bit address scheme
**                      +--------- GCAEN    =0  : General call address disabled
*/


/***********************************************************************************************\
* Public type definitions
\***********************************************************************************************/

/***********************************************************************************************\
* Public memory declarations
\***********************************************************************************************/

/***********************************************************************************************\
* Public prototypes
\***********************************************************************************************/

void IIC_RegWrite(byte address, byte reg,byte val);
byte IIC_RegRead(byte address, byte reg);
void IIC_RegWriteN(byte address, byte reg1,byte N,byte *array);
void IIC_RegReadN(byte address, byte reg1,byte N,byte *array);


#endif  /* _IIC_H_ */
