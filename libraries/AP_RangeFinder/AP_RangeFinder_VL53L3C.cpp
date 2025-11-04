/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  VL53L3C ToF rangefinder driver using bare ST driver

  This driver uses the ST VL53L3C bare driver for accurate histogram-based
  ranging with multi-target detection capability.
*/

#include "AP_RangeFinder_VL53L3C.h"

#if AP_RANGEFINDER_VL53L3C_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Common/AP_Common.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// Include bare driver API
extern "C" {
#include "vl53l3c_driver/vl53lx_api.h"
#include "vl53l3c_driver/vl53lx_platform_ardupilot.h"
}

// Helper macro to cast opaque pointer to bare driver device pointer
#define VL53L_DEV ((VL53LX_Dev_t*)vl53lx_dev)

/*
   The constructor also initializes the rangefinder. Note that this
   constructor is not called until detect() returns a pointer to this driver
*/
AP_RangeFinder_VL53L3C::AP_RangeFinder_VL53L3C(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(std::move(_dev))
    , sum_mm(0)
    , counter(0)
    , mode(DistanceMode::Unknown)
    , vl53lx_dev(nullptr)
{
}

/*
   Detect if a VL53L3C rangefinder is connected. We'll detect by trying to
   read the model ID which should be 0xEAAA.
*/
AP_RangeFinder_Backend *AP_RangeFinder_VL53L3C::detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev, DistanceMode mode)
{
    if (!_dev) {
        return nullptr;
    }

    AP_RangeFinder_VL53L3C *sensor = NEW_NOTHROW AP_RangeFinder_VL53L3C(_state, _params, std::move(_dev));

    if (!sensor) {
        return nullptr;
    }

    sensor->mode = mode;

    // Take semaphore for initialization (ArduPilot standard pattern)
    sensor->dev->get_semaphore()->take_blocking();

    // Check product ID and initialize sensor
    if (!sensor->check_id() || !sensor->init(mode)) {
        sensor->dev->get_semaphore()->give();
        delete sensor;
        return nullptr;
    }

    sensor->dev->get_semaphore()->give();

    return sensor;
}

/*
  check sensor ID
 */
bool AP_RangeFinder_VL53L3C::check_id(void)
{
    uint16_t model_id = 0;
    if (!read_register16(IDENTIFICATION__MODEL_ID, model_id)) {
        DEV_PRINTF("VL53L3C: Failed to read MODEL_ID register\n");
        return false;
    }

    DEV_PRINTF("VL53L3C: Read MODEL_ID=0x%04X (expected 0x%04X)\n", model_id, VL53L3C_PRODUCT_ID);

    if (model_id != VL53L3C_PRODUCT_ID) {
        DEV_PRINTF("VL53L3C: MODEL_ID mismatch!\n");
        return false;
    }

    DEV_PRINTF("VL53L3C: Sensor detected successfully\n");
    return true;
}

/*
  initialise sensor
 */
bool AP_RangeFinder_VL53L3C::init(DistanceMode distance_mode)
{
    // Allocate bare driver device structure
    vl53lx_dev = NEW_NOTHROW VL53LX_Dev_t;
    if (vl53lx_dev == nullptr) {
        return false;
    }

    // Initialize device structure
    memset(vl53lx_dev, 0, sizeof(VL53LX_Dev_t));
    VL53L_DEV->i2c_slave_address = dev->get_bus_address();
    VL53L_DEV->i2c_device = dev.get();  // Store I2CDevice pointer for platform layer

    // Wait for device to boot
    VL53LX_Error status = VL53LX_WaitDeviceBooted(VL53L_DEV);
    if (status != VL53LX_ERROR_NONE) {
        DEV_PRINTF("VL53L3C: WaitDeviceBooted failed: %d\n", status);
        return false;
    }

    // Initialize the device
    status = VL53LX_DataInit(VL53L_DEV);
    if (status != VL53LX_ERROR_NONE) {
        DEV_PRINTF("VL53L3C: DataInit failed: %d\n", status);
        return false;
    }

    // Set distance mode
    VL53LX_DistanceModes vl53_mode;
    switch (distance_mode) {
        case DistanceMode::Medium:
            vl53_mode = VL53LX_DISTANCEMODE_MEDIUM;
            break;
        case DistanceMode::Long:
            vl53_mode = VL53LX_DISTANCEMODE_LONG;
            break;
        default:
            return false;
    }

    status = VL53LX_SetDistanceMode(VL53L_DEV, vl53_mode);
    if (status != VL53LX_ERROR_NONE) {
        DEV_PRINTF("VL53L3C: SetDistanceMode failed: %d\n", status);
        return false;
    }

    // Set measurement timing budget
    status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(VL53L_DEV, TIMING_BUDGET_US);
    if (status != VL53LX_ERROR_NONE) {
        DEV_PRINTF("VL53L3C: SetMeasurementTimingBudget failed: %d\n", status);
        return false;
    }

    // Start continuous ranging
    status = VL53LX_StartMeasurement(VL53L_DEV);
    if (status != VL53LX_ERROR_NONE) {
        DEV_PRINTF("VL53L3C: StartMeasurement failed: %d\n", status);
        return false;
    }

    // Register periodic timer callback
    dev->register_periodic_callback(MEASUREMENT_TIME_MS * 1000UL, FUNCTOR_BIND_MEMBER(&AP_RangeFinder_VL53L3C::timer, void));

    return true;
}

/*
  timer called at measurement rate
  Note: I2C operations in get_reading() are automatically protected by register_periodic_callback()
  We only need to protect internal data (sum_mm, counter) with our own semaphore.
 */
void AP_RangeFinder_VL53L3C::timer()
{
    uint16_t reading_mm;
    if (get_reading(reading_mm)) {
        WITH_SEMAPHORE(_sem);
        sum_mm += reading_mm;
        counter++;
    }
}

/*
  get a reading from the sensor
 */
bool AP_RangeFinder_VL53L3C::get_reading(uint16_t &reading_mm)
{
    if (vl53lx_dev == nullptr) {
        return false;
    }

    // Check if data is ready
    uint8_t data_ready = 0;
    VL53LX_Error status = VL53LX_GetMeasurementDataReady(VL53L_DEV, &data_ready);
    if (status != VL53LX_ERROR_NONE || !data_ready) {
        return false;
    }

    // Get ranging measurement data
    VL53LX_MultiRangingData_t ranging_data;
    status = VL53LX_GetMultiRangingData(VL53L_DEV, &ranging_data);
    if (status != VL53LX_ERROR_NONE) {
        // Clear interrupt and start next measurement
        (void)VL53LX_ClearInterruptAndStartMeasurement(VL53L_DEV);
        return false;
    }

    // Clear interrupt and start next measurement
    status = VL53LX_ClearInterruptAndStartMeasurement(VL53L_DEV);
    if (status != VL53LX_ERROR_NONE) {
        DEV_PRINTF("VL53L3C: ClearInterruptAndStartMeasurement failed: %d\n", status);
    }

    // Check if we have valid targets
    if (ranging_data.NumberOfObjectsFound == 0) {
        return false;
    }

    // Use the first valid target with acceptable range status
    // Range status == 0 means fully valid, but we accept several status codes
    // that indicate usable measurements
    for (uint8_t i = 0; i < ranging_data.NumberOfObjectsFound && i < VL53LX_MAX_RANGE_RESULTS; i++) {
        const VL53LX_TargetRangeData_t *target = &ranging_data.RangeData[i];

        // Check range status
        // Status 0, 4-7, 9 are considered valid measurements
        // 0 = Valid, 4 = Out of bounds, 5-7 = Phase/sigma checks (still usable)
        // 9 = Range complete
        if (target->RangeStatus == 0 ||
            (target->RangeStatus >= 4 && target->RangeStatus <= 7) ||
            target->RangeStatus == 9 ||
            target->RangeStatus == 19 ||  // RANGECOMPLETE_NO_WRAP_CHECK
            target->RangeStatus == 22) {  // RANGECOMPLETE_MERGED_PULSE

            int16_t range_mm = target->RangeMilliMeter;

            // Sanity check the range
            if (range_mm > 0 && range_mm < 5000) {
                reading_mm = (uint16_t)range_mm;
                return true;
            }
        }
    }

    return false;
}

/*
  update state - called from main thread
  Protected by semaphore since timer() thread also accesses sum_mm and counter
 */
void AP_RangeFinder_VL53L3C::update(void)
{
    WITH_SEMAPHORE(_sem);

    if (counter > 0) {
        // Take average of accumulated measurements
        uint16_t avg_mm = sum_mm / counter;
        sum_mm = 0;
        counter = 0;

        // Update state with new measurement
        state.distance_m = avg_mm * 0.001f;
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 300) {
        // No new data for 300ms, set to out of range
        set_status(RangeFinder::Status::OutOfRangeLow);
    }
}

/*
  read a 16 bit register value (for detection only)
  VL53L3C uses 16-bit register addresses, so we must use transfer() instead of read_registers()

  Note: This function is called from detect() which already holds the I2C semaphore,
  so we don't need to take it again here (ArduPilot standard pattern).
 */
bool AP_RangeFinder_VL53L3C::read_register16(uint16_t reg, uint16_t &value)
{
    // VL53L3C uses 16-bit big-endian register addresses
    uint8_t reg_addr[2];
    reg_addr[0] = (reg >> 8) & 0xFF;  // MSB
    reg_addr[1] = reg & 0xFF;          // LSB

    uint8_t buff[2];

    if (!dev->transfer(reg_addr, sizeof(reg_addr), buff, sizeof(buff))) {
        return false;
    }

    value = be16toh_ptr(buff);
    return true;
}

#endif // AP_RANGEFINDER_VL53L3C_ENABLED
