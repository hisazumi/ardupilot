#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_VL53L3C_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/OwnPtr.h>

class AP_RangeFinder_VL53L3C : public AP_RangeFinder_Backend
{

public:
    // VL53L3C supports Medium and Long distance modes
    enum class DistanceMode { Medium, Long, Unknown };

    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev, DistanceMode mode);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // VL53L3C Product ID
    static constexpr uint16_t VL53L3C_PRODUCT_ID = 0xEAAA;

    // Timing budget and measurement period
    static constexpr uint32_t MEASUREMENT_TIME_MS = 50;  // 20Hz update rate
    static constexpr uint32_t TIMING_BUDGET_US = 33000;  // 33ms

    // Register addresses for initial detection
    enum regAddr : uint16_t
    {
        IDENTIFICATION__MODEL_ID = 0x010F,
    };

    // Constructor
    AP_RangeFinder_VL53L3C(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev);

    // Initialization and configuration
    bool init(DistanceMode mode) WARN_IF_UNUSED;
    bool check_id(void);

    // Measurement functions
    void timer();
    bool get_reading(uint16_t &reading_mm);

    // I2C register access (for detection only)
    bool read_register16(uint16_t reg, uint16_t &value) WARN_IF_UNUSED;

    // Member variables
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
    uint32_t sum_mm;
    uint32_t counter;
    DistanceMode mode;

    // Bare driver device structure (opaque pointer, actual type defined in implementation)
    void *vl53lx_dev;

    // Semaphore for protecting internal data (sum_mm, counter)
    HAL_Semaphore _sem;
};

#endif  // AP_RANGEFINDER_VL53L3C_ENABLED
