//*****************************************************************************
//! @file       acc_bma250.h
//! @brief      Header file for accelerometer BMA250. Note that this header
//!             file does not include defines for all BMA250 register addresses.
//!
//! Datasheet   http://www.bosch-sensortec.com/content/language1/downloads/BST-BMA250-DS002-04.pdf
//!
//! Revised     $Date: 2013-04-11 19:41:57 +0200 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9707 $
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/
#ifndef __ACC_BMA250_H__
#define __ACC_BMA250_H__


/******************************************************************************
* If building with a C++ compiler, make all of the definitions in this header
* have a C binding.
******************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif


/******************************************************************************
 * INCLUDES
 */
#include "board.h"


/******************************************************************************
 * DEFINES
 */

// BMA250 addressing space
#define ACC_CHIPID                  0x00    //!< Always 0x03 for BMA250
#define ACC_X_LSB                   0x02    //!< ACC_X_LSB[7:6] = 2 LSb of X data
#define ACC_X_MSB                   0x03    //!< ACC_X_MSB[7:0] = 8 MSb of X data
#define ACC_Y_LSB                   0x04
#define ACC_Y_MSB                   0x05
#define ACC_Z_LSB                   0x06
#define ACC_Z_MSB                   0x07
#define ACC_TEMP                    0x08    //!< Temperature data
#define ACC_INT_STATUS0             0x09    //!< Interrupt status register
#define ACC_INT_STATUS1             0x0A
#define ACC_TAP_SLOPE_INT_STATUS    0x0B    //!< Interrupt detailed status
#define ACC_FLAT_ORIENT_STATUS      0x0C    //!< Interrupt detailed status
#define ACC_RANGE                   0x0F    //!< Working range (2/4/8/16 g)
#define ACC_BW                      0x10    // Filtered bandwidth
#define ACC_PWR_MODE                0x11    // Power configuration
#define ACC_DATA_CFG                0x12
#define ACC_CONF_FILT_SHADOW        0x13
#define ACC_SOFTRESET               0x14    //!< Write 0xB6 for soft reset
#define ACC_INT_EN0                 0x16    // Interrupt enable register
#define ACC_INT_EN1                 0x17
#define ACC_INT1_MAP                0x19    // Interrupt - IO mapping
#define ACC_INT_MAP                 0x1A
#define ACC_INT2_MAP                0x1B
#define ACC_INT_SRC                 0x1E
#define ACC_INT_PIN_CFG             0x20
#define ACC_TAP_INT_TIMING          0x2A
#define ACC_TAP_INT_SAMP_TH         0x2B

// Defines for bit fields in the ACC_INT_STATUS0 register
#define ACC_INT_STATUS0_FLAT_INT    0x80
#define ACC_INT_STATUS0_ORIENT_INT  0x40
#define ACC_INT_STATUS0_S_TAP_INT   0x20
#define ACC_INT_STATUS0_D_TAP_INT   0x10
#define ACC_INT_STATUS0_SLOPE_INT   0x04
#define ACC_INT_STATUS0_HIGH_INT    0x02
#define ACC_INT_STATUS0_LOW_INT     0x01

// Defines for bit fields in the ACC_INT_STATUS1 register
#define ACC_INT_STATUS1_DATA_INT    0x80

// Defines for bit fields in the ACC_RANGE register
#define ACC_RANGE_M                 0x0F
#define ACC_RANGE_2G                0x03    //!<  3.91 mg/LSB
#define ACC_RANGE_4G                0x05    //!<  7.81 mg/LSB
#define ACC_RANGE_8G                0x08    //!< 15.62 mg/LSB
#define ACC_RANGE_16G               0x0C    //!< 31.25 mg/LSB

// Defines for bit fields in the ACC_POWER_MODE register
#define ACC_SUSPEND                 0x80    //!< Set in suspend mode (default 0)
#define ACC_LOWPOWER_EN             0x40    //!< Enable low-power mode (default 0)
#define ACC_SLEEP_DUR_M             0x1E    //!< sleep_dur bitmask
#define ACC_SLEEP_DUR_0MS           0x00    // sleep_dur default value
#define ACC_SLEEP_DUR_0_5MS         0x14
#define ACC_SLEEP_DUR_1MS           0x18
#define ACC_SLEEP_DUR_2MS           0x1C
#define ACC_SLEEP_DUR_4MS           0x20
#define ACC_SLEEP_DUR_6MS           0x24
#define ACC_SLEEP_DUR_10MS          0x28
#define ACC_SLEEP_DUR_25MS          0x2C
#define ACC_SLEEP_DUR_50MS          0x30
#define ACC_SLEEP_DUR_100MS         0x34
#define ACC_SLEEP_DUR_500MS         0x38
#define ACC_SLEEP_DUR_1000MS        0x3C

// Defines for bit fields in the ACC_DATA_CFG register
#define ACC_DATA_UNFILTERED         0x80    // Select unfiltered acceleration
                                            // to be written into data regs.
                                            // Default is 0 (filtered data)
#define ACC_DATA_SHADOW_DIS         0x80    // Disable shadownig procedure.
                                            // Default is 0 (shadowing enabled)

// Defines for bit fields in the ACC_TAP_INT_TIMING register
#define ACC_TAP_QUIET_M             0x80
#define ACC_TAP_QUIET_30MS          0x00
#define ACC_TAP_QUIET_20MS          0x80
#define ACC_TAP_SHOCK_M             0x40
#define ACC_TAP_SHOCK_50MS          0x00
#define ACC_TAP_SHOCK_75MS          0x40
#define ACC_TAP_DUR_M               0x07
#define ACC_TAP_DUR_50MS            0x00
#define ACC_TAP_DUR_100MS           0x01
#define ACC_TAP_DUR_150MS           0x02
#define ACC_TAP_DUR_200MS           0x03
#define ACC_TAP_DUR_250MS           0x04
#define ACC_TAP_DUR_375MS           0x05
#define ACC_TAP_DUR_500MS           0x06
#define ACC_TAP_DUR_700MS           0x07

// Defines for bit fields in the ACC_TAP_INT_SAMP_TH register
#define ACC_TAMP_SAMP_M             0xC0
#define ACC_TAP_SAMP_2_SAMPLES      0x00    // The number of samples to be
#define ACC_TAP_SAMP_4_SAMPLES      0x40    // processed after wakeup in
#define ACC_TAP_SAMP_8_SAMPLES      0x80    // low-power mode.
#define ACC_TAP_SAMP_16_SAMPLES     0xC0
#define ACC_TAP_TH_M                0x1F    // Bitmask for tap_th subset of
                                            // register.
                                            // The LSb of tap_th
                                            // corresponds to the following
                                            // acceleration difference:
                                            // 2g  range: 62.5mg
                                            // 4g  range: 125mg
                                            // 8g  range: 250mg
                                            // 16g range: 500mg

// Defines for bit fields in the ACC_BW register
// delta_t = time between successive acc samples
#define ACC_BW_7_81HZ               0x08    //!< delta_t = 64   ms
#define ACC_BW_15_63HZ              0x09    //!< delta_t = 32   ms
#define ACC_BW_31_25HZ              0x0A    //!< delta_t = 16   ms
#define ACC_BW_62_5HZ               0x0B    //!< delta_t =  8   ms
#define ACC_BW_125HZ                0x0C    //!< delta_t =  4   ms
#define ACC_BW_250HZ                0x0D    //!< delta_t =  2   ms
#define ACC_BW_500HZ                0x0E    //!< delta_t =  1   ms
#define ACC_BW_1000HZ               0x0F    //!< delta_t =  0.5 ms

// Defines for bit fields in the ACC_INT_EN0 register
#define ACC_INT_EN0_FLAT_EN         0x80
#define ACC_INT_EN0_ORIENT_EN       0x40
#define ACC_INT_EN0_S_TAP_EN        0x20
#define ACC_INT_EN0_D_TAP_EN        0x10
#define ACC_INT_EN0_SLOPE_Z_EN      0x04
#define ACC_INT_EN0_SLOPE_Y_EN      0x02
#define ACC_INT_EN0_SLOPE_X_EN      0x01

// Defines for bit fields in the ACC_INT_EN1 register
#define ACC_INT_EN1_DATA_EN         0x10
#define ACC_INT_EN1_LOW_EN          0x08
#define ACC_INT_EN1_HIGH_Z_EN       0x04
#define ACC_INT_EN1_HIGH_Y_EN       0x02
#define ACC_INT_EN1_HIGH_X_EN       0x01

// Defines for bit fields in the ACC_INT1_MAP and ACC_INT2_MAP registers
#define ACC_INT_MAP_FLAT            0x80
#define ACC_INT_MAP_ORIENT          0x40
#define ACC_INT_MAP_S_TAP           0x20
#define ACC_INT_MAP_D_TAP           0x10
#define ACC_INT_MAP_SLOPE           0x04
#define ACC_INT_MAP_HIGH            0x02
#define ACC_INT_MAP_LOW             0x01

// Defines for bit fields in the ACC_INT_MAP register
#define ACC_INT_MAP_DATA_INT1       0x01    // New data IRQ to pin INT1
#define ACC_INT_MAP_DATA_INT2       0x80    // New data IRQ to pin INT2

// Defines for bit fields in the ACC_INT_SRC register
#define ACC_INT_SRC_DATA_FILT       0x20
#define ACC_INT_SRC_TAP_FILT        0x01
#define ACC_INT_SRC_SLOPE_FILT      0x04
#define ACC_INT_SRC_HIGH_FILT       0x02
#define ACC_INT_SRC_LOW_FILT        0x01

// Defines for bit fields in the ACC_INT_PIN_CFG register
#define ACC_INT_CFG_INT2_OD         0x08    // Select open drive for INT2 pin
#define ACC_INT_CFG_INT2_PUSH_PULL  0x00    // Select open drive for INT1 pin
#define ACC_INT_CFG_INT2_ACTIVE_HI  0x04    // Select active high for INT2 pin
#define ACC_INT_CFG_INT2_ACTIVE_LO  0x00    // Select active low for INT2 pin
#define ACC_INT_CFG_INT1_OD         0x02    // Select open drive for INT1 pin
#define ACC_INT_CFG_INT1_PUSH_PULL  0x00    // Select open drive for INT1 pin
#define ACC_INT_CFG_INT1_ACTIVE_HI  0x01    // Select active high for INT1 pin
#define ACC_INT_CFG_INT1_ACTIVE_LO  0x00    // Select active high for INT1 pin

// Perform soft reset
#define ACC_SOFTRESET_EN            0xB6    // Soft reset by writing 0xB6 to
                                            // ACC_SOFTRESET

// R/W Bitmask
#define ACC_READ_M                  0x80    //!< Read action bitmask


/******************************************************************************
* FUNCTION PROTOTYPES
*/

extern void accInit(void);
extern void accDisable(void);
extern void accReadReg(uint8_t ui8Addr, uint8_t *pui8Buf, uint8_t ui8Len);
extern void accWriteReg(uint8_t ui8Addr, const uint8_t *pui8Buf,
                        uint8_t ui8Len);
extern void accReadData(int16_t *pi16XVal, int16_t *pi16YVal,
                        int16_t *pi16ZVal);
extern void accIntRegister(uint8_t ui8Pins, void (*pfnHandler)(void));
extern void accIntUnregister(uint8_t ui8Pins);
extern void accIntEnable(uint8_t ui8Pins);
extern void accIntDisable(uint8_t ui8Pins);
extern void accIntClear(uint8_t ui8Pins);
extern void accIntTypeSet(uint8_t ui8Pins, uint32_t ui32IntType);


/******************************************************************************
* Mark the end of the C bindings section for C++ compilers.
******************************************************************************/
#ifdef __cplusplus
}
#endif
#endif /* __ACC_BMA250_H__ */
