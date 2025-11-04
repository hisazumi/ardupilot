/*******************************************************************************
 * ArduPilot Platform Abstraction Layer for VL53L3C
 *
 * This file provides the platform-specific functions required by the VL53L3C
 * bare driver, implemented using ArduPilot HAL.
 *
 * Based on STMicroelectronics VL53LX platform layer
 ******************************************************************************/

#ifndef _VL53LX_PLATFORM_ARDUPILOT_H_
#define _VL53LX_PLATFORM_ARDUPILOT_H_

#include "vl53lx_platform_log.h"
#include "vl53lx_ll_def.h"

#define VL53LX_IPP_API
#include "vl53lx_platform_ipp_imports.h"
#include "vl53lx_platform_user_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise platform comms
 */
VL53LX_Error VL53LX_CommsInitialise(
    VL53LX_Dev_t *pdev,
    uint8_t comms_type,
    uint16_t comms_speed_khz);

/**
 * @brief Close platform comms
 */
VL53LX_Error VL53LX_CommsClose(VL53LX_Dev_t *pdev);

/**
 * @brief Write multiple bytes to the device
 */
VL53LX_Error VL53LX_WriteMulti(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint8_t *pdata,
    uint32_t count);

/**
 * @brief Read multiple bytes from the device
 */
VL53LX_Error VL53LX_ReadMulti(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint8_t *pdata,
    uint32_t count);

/**
 * @brief Write a single byte to the device
 */
VL53LX_Error VL53LX_WrByte(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint8_t data);

/**
 * @brief Write a single word (16-bit) to the device
 */
VL53LX_Error VL53LX_WrWord(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint16_t data);

/**
 * @brief Write a single dword (32-bit) to the device
 */
VL53LX_Error VL53LX_WrDWord(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint32_t data);

/**
 * @brief Read a single byte from the device
 */
VL53LX_Error VL53LX_RdByte(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint8_t *pdata);

/**
 * @brief Read a single word (16-bit) from the device
 */
VL53LX_Error VL53LX_RdWord(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint16_t *pdata);

/**
 * @brief Read a single dword (32-bit) from the device
 */
VL53LX_Error VL53LX_RdDWord(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint32_t *pdata);

/**
 * @brief Wait for specified microseconds
 */
VL53LX_Error VL53LX_WaitUs(
    VL53LX_Dev_t *pdev,
    int32_t wait_us);

/**
 * @brief Wait for specified milliseconds
 */
VL53LX_Error VL53LX_WaitMs(
    VL53LX_Dev_t *pdev,
    int32_t wait_ms);

/**
 * @brief Get timer frequency
 */
VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz);

/**
 * @brief Get timer value
 */
VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count);

/**
 * @brief Get current tick count in milliseconds
 */
VL53LX_Error VL53LX_GetTickCount(
    VL53LX_DEV Dev,
    uint32_t *ptime_ms);

/**
 * @brief Wait for register value with mask
 */
VL53LX_Error VL53LX_WaitValueMaskEx(
    VL53LX_Dev_t *pdev,
    uint32_t timeout_ms,
    uint16_t index,
    uint8_t value,
    uint8_t mask,
    uint32_t poll_delay_ms);


// GPIO functions (stubs for ArduPilot - not typically used)
VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode);
VL53LX_Error VL53LX_GpioSetValue(uint8_t pin, uint8_t value);
VL53LX_Error VL53LX_GpioGetValue(uint8_t pin, uint8_t* pvalue);
VL53LX_Error VL53LX_GpioXshutdown(uint8_t value);
VL53LX_Error VL53LX_GpioCommsSelect(uint8_t value);
VL53LX_Error VL53LX_GpioPowerEnable(uint8_t value);
VL53LX_Error VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge_type);
VL53LX_Error VL53LX_GpioInterruptDisable(void);

// IPP histogram processing stub (Intel IPP not used in ArduPilot)
VL53LX_Error VL53LX_ipp_hist_process_data(
    VL53LX_DEV Dev,
    VL53LX_dmax_calibration_data_t    *pdmax_cal,
    VL53LX_hist_gen3_dmax_config_t    *pdmax_cfg,
    VL53LX_hist_post_process_config_t *ppost_cfg,
    VL53LX_histogram_bin_data_t       *pbins,
    VL53LX_xtalk_histogram_data_t     *pxtalk,
    uint8_t                           *pArea1,
    uint8_t                           *pArea2,
    uint8_t                           *HistMergeNumber,
    VL53LX_range_results_t            *presults);

#ifdef __cplusplus
}
#endif

#endif // _VL53LX_PLATFORM_ARDUPILOT_H_
