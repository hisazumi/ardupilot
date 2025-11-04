/*******************************************************************************
 * ArduPilot Platform User Data for VL53L3C
 *
 * Based on STMicroelectronics VL53LX platform user data
 *
 * This file defines the device structure and data access macros for the
 * VL53L3C bare driver when used with ArduPilot.
 ******************************************************************************/

#ifndef _VL53LX_PLATFORM_USER_DATA_H_
#define _VL53LX_PLATFORM_USER_DATA_H_

#ifndef __KERNEL__
#include <stdlib.h>
#endif

#include "vl53lx_def.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief VL53LX Device structure for ArduPilot
 *
 * This structure contains the device state and platform-specific data.
 * For ArduPilot, the I2C device pointer is stored as a void* to allow
 * C code to access it, while the actual AP_HAL::I2CDevice object is
 * managed in C++.
 */
typedef struct {
    VL53LX_DevData_t Data;           /*!< Low Level Driver data structure */
    uint8_t i2c_slave_address;       /*!< I2C slave address */
    uint8_t comms_type;              /*!< Communication type (I2C) */
    uint16_t comms_speed_khz;        /*!< Communication speed in kHz */

    // ArduPilot HAL I2C device pointer (set from C++)
    void *i2c_device;                /*!< Pointer to AP_HAL::I2CDevice */

    uint8_t   I2cDevAddr;
    int     Present;
    int     Enabled;
    int     LoopState;
    int     FirstStreamCountZero;
    int     Idle;
    int     Ready;
    uint8_t RangeStatus;
    FixPoint1616_t SignalRateRtnMegaCps;
    VL53LX_DeviceState device_state; /*!< Device State */
} VL53LX_Dev_t;

typedef VL53LX_Dev_t* VL53LX_DEV;

/**
 * @brief Get ST private structure data access
 *
 * @param Dev       Device Handle
 * @param field     ST structure field name
 */
#define VL53LXDevDataGet(Dev, field) (Dev->Data.field)

/**
 * @brief Set ST private structure data field
 *
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * @param data      Data to be set
 */
#define VL53LXDevDataSet(Dev, field, data) ((Dev->Data.field) = (data))

/**
 * @brief Get low level driver handle
 */
#define VL53LXDevStructGetLLDriverHandle(Dev) (&Dev->Data.LLData)

/**
 * @brief Get low level results handle
 */
#define VL53LXDevStructGetLLResultsHandle(Dev) (&Dev->Data.llresults)

// Legacy PAL macros (for compatibility)
#define PALDevDataGet(Dev, field) (Dev->Data.field)
#define PALDevDataSet(Dev, field, data) ((Dev->Data.field) = (data))

#ifdef __cplusplus
}
#endif

#endif  /* _VL53LX_PLATFORM_USER_DATA_H_ */
