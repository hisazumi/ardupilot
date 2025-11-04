/*******************************************************************************
 * ArduPilot Platform Abstraction Layer for VL53L3C - Implementation
 *
 * This file provides the platform-specific functions required by the VL53L3C
 * bare driver, implemented using ArduPilot HAL.
 *
 * Based on STMicroelectronics VL53LX platform layer
 ******************************************************************************/

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

// Include bare driver headers for complete type definitions
#include "vl53l3c_driver/vl53lx_ll_def.h"
#include "vl53l3c_driver/vl53lx_platform_ardupilot.h"
#include "vl53l3c_driver/vl53lx_hist_funcs.h"

// External AP_HAL instance
extern const AP_HAL::HAL& hal;

// Wrap all platform functions in extern "C" for C linkage
extern "C" {

/**
 * @brief Initialise platform comms
 */
VL53LX_Error VL53LX_CommsInitialise(
    VL53LX_Dev_t *pdev,
    uint8_t comms_type,
    uint16_t comms_speed_khz)
{
    pdev->comms_type = comms_type;
    pdev->comms_speed_khz = comms_speed_khz;
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Close platform comms
 */
VL53LX_Error VL53LX_CommsClose(VL53LX_Dev_t *pdev)
{
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Write multiple bytes to the device
 *
 * Writes a buffer to the I2C device. The register index is sent as
 * a 16-bit big-endian value before the data buffer.
 */
VL53LX_Error VL53LX_WriteMulti(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint8_t *pdata,
    uint32_t count)
{
    if (pdev->i2c_device == nullptr) {
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }

    // Cast i2c_device back to AP_HAL::I2CDevice*
    AP_HAL::I2CDevice *dev = static_cast<AP_HAL::I2CDevice *>(pdev->i2c_device);

    // The VL53L3C uses 16-bit register addresses in big-endian format
    // We need to send: [index_hi, index_lo, data[0], data[1], ...]
    // Maximum transfer size is typically < 256 bytes for VL53L3C
    uint8_t buffer[258];  // 2 bytes for register address + max 256 data bytes

    if (count > 256) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }

    buffer[0] = (index >> 8) & 0xFF;  // High byte
    buffer[1] = index & 0xFF;         // Low byte

    // Copy data to buffer
    for (uint32_t i = 0; i < count; i++) {
        buffer[i + 2] = pdata[i];
    }

    // Perform I2C write
    if (!dev->transfer(buffer, count + 2, nullptr, 0)) {
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }

    return VL53LX_ERROR_NONE;
}

/**
 * @brief Read multiple bytes from the device
 *
 * Reads a buffer from the I2C device. First writes the 16-bit register
 * index, then reads the requested number of bytes.
 */
VL53LX_Error VL53LX_ReadMulti(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint8_t *pdata,
    uint32_t count)
{
    if (pdev->i2c_device == nullptr) {
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }

    // Cast i2c_device back to AP_HAL::I2CDevice*
    AP_HAL::I2CDevice *dev = static_cast<AP_HAL::I2CDevice *>(pdev->i2c_device);

    // Send 16-bit register address in big-endian format
    uint8_t reg_addr[2];
    reg_addr[0] = (index >> 8) & 0xFF;  // High byte
    reg_addr[1] = index & 0xFF;         // Low byte

    // Perform I2C write-read transaction
    if (!dev->transfer(reg_addr, 2, pdata, count)) {
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }

    return VL53LX_ERROR_NONE;
}

/**
 * @brief Write a single byte to the device
 */
VL53LX_Error VL53LX_WrByte(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint8_t data)
{
    return VL53LX_WriteMulti(pdev, index, &data, 1);
}

/**
 * @brief Write a single word (16-bit) to the device
 *
 * Manages big-endian nature of device (MS byte written first)
 */
VL53LX_Error VL53LX_WrWord(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint16_t data)
{
    uint8_t buffer[2];
    buffer[0] = (data >> 8) & 0xFF;  // MS byte
    buffer[1] = data & 0xFF;         // LS byte
    return VL53LX_WriteMulti(pdev, index, buffer, 2);
}

/**
 * @brief Write a single dword (32-bit) to the device
 *
 * Manages big-endian nature of device (MS byte written first)
 */
VL53LX_Error VL53LX_WrDWord(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint32_t data)
{
    uint8_t buffer[4];
    buffer[0] = (data >> 24) & 0xFF;  // MS byte
    buffer[1] = (data >> 16) & 0xFF;
    buffer[2] = (data >> 8) & 0xFF;
    buffer[3] = data & 0xFF;          // LS byte
    return VL53LX_WriteMulti(pdev, index, buffer, 4);
}

/**
 * @brief Read a single byte from the device
 */
VL53LX_Error VL53LX_RdByte(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint8_t *pdata)
{
    return VL53LX_ReadMulti(pdev, index, pdata, 1);
}

/**
 * @brief Read a single word (16-bit) from the device
 *
 * Manages big-endian nature of device (MS byte read first)
 */
VL53LX_Error VL53LX_RdWord(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint16_t *pdata)
{
    uint8_t buffer[2];
    VL53LX_Error status = VL53LX_ReadMulti(pdev, index, buffer, 2);
    if (status == VL53LX_ERROR_NONE) {
        *pdata = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    }
    return status;
}

/**
 * @brief Read a single dword (32-bit) from the device
 *
 * Manages big-endian nature of device (MS byte read first)
 */
VL53LX_Error VL53LX_RdDWord(
    VL53LX_Dev_t *pdev,
    uint16_t index,
    uint32_t *pdata)
{
    uint8_t buffer[4];
    VL53LX_Error status = VL53LX_ReadMulti(pdev, index, buffer, 4);
    if (status == VL53LX_ERROR_NONE) {
        *pdata = ((uint32_t)buffer[0] << 24) |
                 ((uint32_t)buffer[1] << 16) |
                 ((uint32_t)buffer[2] << 8) |
                 (uint32_t)buffer[3];
    }
    return status;
}

/**
 * @brief Wait for specified microseconds
 */
VL53LX_Error VL53LX_WaitUs(
    VL53LX_Dev_t *pdev,
    int32_t wait_us)
{
    (void)pdev;  // Unused parameter
    hal.scheduler->delay_microseconds(wait_us);
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Wait for specified milliseconds
 */
VL53LX_Error VL53LX_WaitMs(
    VL53LX_Dev_t *pdev,
    int32_t wait_ms)
{
    (void)pdev;  // Unused parameter
    hal.scheduler->delay(wait_ms);
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Get timer frequency
 *
 * Returns 1kHz (1000 Hz) for millisecond-based timing
 */
VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    *ptimer_freq_hz = 1000;  // 1kHz (millisecond resolution)
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Get timer value
 *
 * Returns current time in milliseconds
 */
VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count)
{
    *ptimer_count = (int32_t)AP_HAL::millis();
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Get current tick count in milliseconds
 */
VL53LX_Error VL53LX_GetTickCount(
    VL53LX_DEV Dev,
    uint32_t *ptime_ms)
{
    (void)Dev;  // Unused parameter
    *ptime_ms = AP_HAL::millis();
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Wait for register value with mask
 *
 * Polls a register until the masked value equals the expected value,
 * or until timeout occurs.
 */
VL53LX_Error VL53LX_WaitValueMaskEx(
    VL53LX_Dev_t *pdev,
    uint32_t timeout_ms,
    uint16_t index,
    uint8_t value,
    uint8_t mask,
    uint32_t poll_delay_ms)
{
    uint32_t start_time_ms = AP_HAL::millis();
    uint8_t register_value = 0;
    VL53LX_Error status = VL53LX_ERROR_NONE;

    // Poll until timeout
    while ((AP_HAL::millis() - start_time_ms) < timeout_ms) {
        status = VL53LX_RdByte(pdev, index, &register_value);
        if (status != VL53LX_ERROR_NONE) {
            return status;
        }

        // Check if masked value matches
        if ((register_value & mask) == value) {
            return VL53LX_ERROR_NONE;
        }

        // Wait before next poll
        if (poll_delay_ms > 0) {
            hal.scheduler->delay(poll_delay_ms);
        }
    }

    // Timeout occurred
    return VL53LX_ERROR_TIME_OUT;
}


//=============================================================================
// GPIO Functions (Stubs - not typically used in ArduPilot)
//=============================================================================

VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode)
{
    (void)pin;
    (void)mode;
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

VL53LX_Error VL53LX_GpioSetValue(uint8_t pin, uint8_t value)
{
    (void)pin;
    (void)value;
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

VL53LX_Error VL53LX_GpioGetValue(uint8_t pin, uint8_t* pvalue)
{
    (void)pin;
    (void)pvalue;
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

VL53LX_Error VL53LX_GpioXshutdown(uint8_t value)
{
    (void)value;
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

VL53LX_Error VL53LX_GpioCommsSelect(uint8_t value)
{
    (void)value;
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

VL53LX_Error VL53LX_GpioPowerEnable(uint8_t value)
{
    (void)value;
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

VL53LX_Error VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
    (void)function;
    (void)edge_type;
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

VL53LX_Error VL53LX_GpioInterruptDisable(void)
{
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

/**
 * @brief IPP histogram processing stub
 *
 * Intel IPP is not used in ArduPilot. This function wraps the standard
 * histogram processing function.
 */
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
    VL53LX_range_results_t            *presults)
{
    (void)Dev;  // Unused parameter
    // Call the non-IPP version
    return VL53LX_hist_process_data(
        pdmax_cal,
        pdmax_cfg,
        ppost_cfg,
        pbins,
        pxtalk,
        pArea1,
        pArea2,
        presults,
        HistMergeNumber);
}

} // extern "C"
