#ifndef RESCUE_ROVER_HARDWARE_PROFILE_H
#define RESCUE_ROVER_HARDWARE_PROFILE_H

/*
 * Confirmed component parameters for the first rescue-rover prototype.
 * Keep installation-dependent values such as effective wheel radius, track
 * width, encoder polarity and sensor mounting geometry out of this file.
 */

/* MG513P30_12V with the Hall AB encoder and 1:30 gearbox. */
#define ROVER_MG513_HALL_PULSES_PER_MOTOR_REVOLUTION 13U
#define ROVER_MG513_GEAR_RATIO 30U
#define ROVER_MG513_QUADRATURE_EDGES_PER_PULSE 4U
#define ROVER_MG513_COUNTS_PER_WHEEL_REVOLUTION \
    (ROVER_MG513_HALL_PULSES_PER_MOTOR_REVOLUTION \
        * ROVER_MG513_GEAR_RATIO \
        * ROVER_MG513_QUADRATURE_EDGES_PER_PULSE)
#define ROVER_MG513_ENCODER_SUPPLY_MIN_MV 3300U
#define ROVER_MG513_ENCODER_SUPPLY_MAX_MV 5000U
#define ROVER_MG513_RATED_CURRENT_MA 360U
#define ROVER_MG513_STALL_CURRENT_MA 3200U

/* MINIF / DVP0501C1 (TinyF) default UART profile. */
#define ROVER_TINYF_DEFAULT_BAUD 115200U
#define ROVER_TINYF_DEFAULT_FRAME_RATE_HZ 30U
#define ROVER_TINYF_MIN_RANGE_MM 20U
#define ROVER_TINYF_MAX_RANGE_MM 4500U
#define ROVER_TINYF_CONFIDENCE_MAX 62U

#endif
