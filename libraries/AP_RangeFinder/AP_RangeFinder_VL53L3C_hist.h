/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Histogram processing structures and definitions for VL53L3C rangefinder
 * Adapted from STMicroelectronics VL53LX bare driver
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 */

#pragma once

#include <stdint.h>

// Constants
#define VL53L3C_MAX_BIN_SEQUENCE_LENGTH  6
#define VL53L3C_HISTOGRAM_BUFFER_SIZE   24
#define VL53L3C_MAX_PULSES              8

// Device state enums
enum class VL53L3C_DeviceState : uint8_t {
    POWERDOWN = 0,
    WAIT_STATICINIT,
    STANDBY,
    IDLE,
    RANGING_DSS_AUTO,
    RANGING_DSS_MANUAL,
    RANGING_WAIT_GPH_SYNC,
};

// Histogram algorithm select
enum class VL53L3C_HistAlgoSelect : uint8_t {
    HIST_ALGO_GEN3 = 0,
    HIST_ALGO_GEN4 = 1,
};

// Histogram target order
enum class VL53L3C_HistTargetOrder : uint8_t {
    STRONGEST_FIRST = 0,
    CLOSEST_FIRST = 1,
};

// Ambient estimation method
enum class VL53L3C_HistAmbEstMethod : uint8_t {
    AMB_EST_THRESHOLDED = 0,
    AMB_EST_ALL_BINS = 1,
};

/**
 * @struct VL53L3C_histogram_bin_data_t
 * @brief Histogram bin data structure
 *
 * Contains histogram bin values and related measurement data
 * This is a simplified version focusing on essential fields for histogram processing
 */
struct VL53L3C_histogram_bin_data_t {
    VL53L3C_DeviceState  cfg_device_state;  // Configured device state
    VL53L3C_DeviceState  rd_device_state;   // Read device state

    uint8_t  zone_id;                       // Zone ID
    uint32_t time_stamp;                    // Measurement timestamp

    // Bin configuration
    uint8_t  vcsel_width;                   // VCSEL pulse width
    uint8_t  vcsel_period_a;               // VCSEL period A
    uint8_t  vcsel_period_b;               // VCSEL period B
    uint8_t  number_of_ambient_bins;        // Number of ambient bins

    uint8_t  bin_seq[VL53L3C_MAX_BIN_SEQUENCE_LENGTH];  // Bin sequence
    uint8_t  bin_rep[VL53L3C_MAX_BIN_SEQUENCE_LENGTH];  // Bin repetition

    int32_t  bin_data[VL53L3C_HISTOGRAM_BUFFER_SIZE];   // 24 histogram bins

    // Result status
    uint8_t  result__interrupt_status;
    uint8_t  result__range_status;
    uint8_t  result__report_status;
    uint8_t  result__stream_count;

    uint16_t result__dss_actual_effective_spads;  // Actual effective SPADs

    // Phase calibration
    uint16_t phasecal_result__reference_phase;
    uint8_t  phasecal_result__vcsel_start;
    uint8_t  cal_config__vcsel_start;

    uint16_t vcsel_width_value;             // VCSEL width value
    uint16_t fast_osc_frequency;            // Fast oscillator frequency

    uint32_t total_periods_elapsed;         // Total periods elapsed
    uint32_t peak_duration_us;              // Peak duration in microseconds
    uint32_t woi_duration_us;               // Window of interest duration

    int32_t  min_bin_value;                 // Minimum bin value
    int32_t  max_bin_value;                 // Maximum bin value

    uint16_t zero_distance_phase;           // Zero distance phase
    uint8_t  number_of_ambient_samples;     // Number of ambient samples
    int32_t  ambient_events_sum;            // Sum of ambient events
    int32_t  ambient_events_avg;            // Average ambient events

    uint8_t  roi_config__user_roi_centre_spad;            // ROI center SPAD
    uint8_t  roi_config__user_roi_requested_global_xy_size;  // ROI size
};

/**
 * @struct VL53L3C_hist_pulse_data_t
 * @brief Pulse data for histogram processing
 *
 * Contains information about a detected pulse in the histogram
 */
struct VL53L3C_hist_pulse_data_t {
    uint8_t  pulse_no;                      // Pulse number
    uint8_t  vcsel_width;                   // VCSEL width
    uint8_t  start_bin;                     // Start bin index
    uint8_t  end_bin;                       // End bin index
    uint8_t  peak_bin;                      // Peak bin index

    uint8_t  min_bin;                       // Minimum bin for this pulse
    uint8_t  max_bin;                       // Maximum bin for this pulse

    int32_t  signal_total_events;           // Total signal events
    int32_t  signal_events_sum;             // Sum of signal events
    int32_t  avg_signal_events;             // Average signal events

    uint32_t min_phase;                     // Minimum phase
    uint32_t avg_phase;                     // Average phase
    uint32_t max_phase;                     // Maximum phase

    uint16_t sigma_mm;                      // Sigma estimate in mm
};

/**
 * @struct VL53L3C_hist_post_process_config_t
 * @brief Histogram post-processing configuration
 *
 * Configuration parameters for histogram processing algorithms
 */
struct VL53L3C_hist_post_process_config_t {
    VL53L3C_HistAlgoSelect   hist_algo_select;     // Algorithm selection
    VL53L3C_HistTargetOrder  hist_target_order;    // Target ordering

    uint8_t   filter_woi0;                  // Filter window of interest 0
    uint8_t   filter_woi1;                  // Filter window of interest 1

    VL53L3C_HistAmbEstMethod hist_amb_est_method;  // Ambient estimation method

    uint8_t   ambient_thresh_sigma0;        // Ambient threshold sigma 0
    uint8_t   ambient_thresh_sigma1;        // Ambient threshold sigma 1
    uint16_t  ambient_thresh_events_scaler; // Ambient threshold events scaler

    int32_t   min_ambient_thresh_events;    // Minimum ambient threshold events
    uint16_t  noise_threshold;              // Noise threshold

    int32_t   signal_total_events_limit;    // Signal total events limit
    uint8_t   sigma_estimator__sigma_ref_mm;  // Sigma reference mm
    uint16_t  sigma_thresh;                 // Sigma threshold

    int16_t   range_offset_mm;              // Range offset in mm
    uint16_t  gain_factor;                  // Gain factor (fixed point 2.14)

    uint8_t   valid_phase_low;              // Valid phase low limit
    uint8_t   valid_phase_high;             // Valid phase high limit

    uint8_t   algo__consistency_check__phase_tolerance;       // Phase tolerance
    uint8_t   algo__consistency_check__event_sigma;           // Event sigma
    uint16_t  algo__consistency_check__event_min_spad_count;  // Min SPAD count
    uint16_t  algo__consistency_check__min_max_tolerance;     // Min/max tolerance
};

/**
 * @struct VL53L3C_range_data_t
 * @brief Range measurement data
 *
 * Contains the final range measurement result
 */
struct VL53L3C_range_data_t {
    uint8_t   range_id;                     // Range ID
    uint32_t  time_stamp;                   // Timestamp

    uint8_t   pulse_no;                     // Pulse number
    uint8_t   vcsel_width;                  // VCSEL width
    uint8_t   start_bin;                    // Start bin
    uint8_t   end_bin;                      // End bin
    uint8_t   peak_bin;                     // Peak bin

    uint8_t   min_bin;                      // Min bin

    uint16_t  width;                        // Width
    uint8_t   woi_width;                    // Window of interest width

    uint16_t  fast_osc_frequency;           // Fast oscillator frequency
    uint16_t  zero_distance_phase;          // Zero distance phase
    uint16_t  actual_effective_spads;       // Actual effective SPADs

    uint32_t  total_periods_elapsed;        // Total periods elapsed
    uint32_t  peak_duration_us;             // Peak duration
    uint32_t  woi_duration_us;              // WOI duration

    uint32_t  signal_total_events;          // Total signal events
    uint32_t  signal_events_sum;            // Signal events sum
    int32_t   avg_signal_events;            // Average signal events

    uint16_t  peak_signal_count_rate_mcps;  // Peak signal count rate
    uint16_t  avg_signal_count_rate_mcps;   // Average signal count rate
    uint16_t  ambient_count_rate_mcps;      // Ambient count rate
    uint16_t  total_rate_per_spad_mcps;     // Total rate per SPAD
    uint32_t  peak_rate_per_spad_kcps;      // Peak rate per SPAD

    uint16_t  sigma_mm;                     // Sigma estimate in mm

    uint16_t  min_phase;                    // Minimum phase
    uint16_t  avg_phase;                    // Average phase
    uint16_t  max_phase;                    // Maximum phase

    int16_t   min_range_mm;                 // Minimum range in mm
    int16_t   median_range_mm;              // Median range in mm
    int16_t   max_range_mm;                 // Maximum range in mm

    uint8_t   range_status;                 // Range status code
};

/**
 * @struct VL53L3C_hist_gen3_algo_private_data_t
 * @brief Gen3 histogram algorithm working data
 *
 * Internal working data structure for Gen3 histogram processing algorithm
 * This structure contains intermediate calculation results
 */
struct VL53L3C_hist_gen3_algo_private_data_t {
    uint8_t  vcsel_width;                   // VCSEL width
    uint8_t  vcsel_period_a;               // VCSEL period A
    uint8_t  vcsel_period_b;               // VCSEL period B
    uint8_t  woi_width;                     // Window of interest width
    uint8_t  ambient_thresh_events_scaler;  // Ambient threshold events scaler

    int32_t  ambient_events_avg;            // Average ambient events
    int32_t  ambient_threshold;             // Ambient threshold

    // Thresholding arrays
    uint8_t  bin_thresholded[VL53L3C_HISTOGRAM_BUFFER_SIZE];      // Thresholded bins
    uint8_t  bin_labelled[VL53L3C_HISTOGRAM_BUFFER_SIZE];         // Labeled bins
    uint8_t  pulse_boundary[VL53L3C_HISTOGRAM_BUFFER_SIZE];       // Pulse boundaries

    // Bin processing arrays
    int32_t  bin_data_ambient_subtracted[VL53L3C_HISTOGRAM_BUFFER_SIZE];  // Ambient subtracted
    int32_t  bin_data_clipped[VL53L3C_HISTOGRAM_BUFFER_SIZE];             // Clipped data
    int32_t  bin_data_filtered[VL53L3C_HISTOGRAM_BUFFER_SIZE];            // Filtered data

    uint8_t  first_bin_above_threshold;     // First bin above threshold
    uint8_t  last_bin_above_threshold;      // Last bin above threshold
    uint8_t  num_bins_above_threshold;      // Number of bins above threshold

    // Pulse detection results
    VL53L3C_hist_pulse_data_t  pulses[VL53L3C_MAX_PULSES];  // Detected pulses

    // Temporary histogram structures for processing
    VL53L3C_histogram_bin_data_t  histogram_avg;            // Averaged histogram
    VL53L3C_histogram_bin_data_t  histogram_raw;            // Raw histogram
    VL53L3C_histogram_bin_data_t  histogram_ambient;        // Ambient histogram
    VL53L3C_histogram_bin_data_t  histogram_removed_ambient;  // Ambient removed
    VL53L3C_histogram_bin_data_t  histogram_pulse;          // Pulse histogram
};
