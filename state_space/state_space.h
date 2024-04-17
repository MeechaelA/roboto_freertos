#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <float.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"
#include "FreeRTOS.h"
#include "BNO055/bno055.h"
#include "drivers/quaternion.h"
#include "drivers/init_imu.h"
#include "drivers/timers.h"
#include "priorities.h"
#include "task.h"

typedef struct {
  uint16_t pwm_base;
  uint16_t pwm_out;
  int8_t max_angle;       // Max Angle of Servo
  int8_t min_angle;       // Min Angle of Servo
  uint32_t max_pwm_value;   // Max PWM Value, use https://1drv.ms/x/s!AjwkVZL4JwUqicdM2-osK8LSTu_taQ?e=FnrZeL
  uint32_t min_pwm_value;   // Min PWM Value, use https://1drv.ms/x/s!AjwkVZL4JwUqicdM2-osK8LSTu_taQ?e=FnrZeL
  uint32_t pwm_frequency;
  uint32_t pwm_clock;
  uint32_t value;
  uint32_t load;
  uint32_t angle;            // Angle of Servo
} servo;

typedef struct {
  uint32_t pwm_base_0;
  uint32_t pwm_out_0;
  uint32_t pwm_gen_0;

  uint32_t pwm_base_1;
  uint32_t pwm_out_1;
  uint32_t pwm_gen_1;

  int32_t max_speed;       // Max Speed of Servo
  int32_t min_speed;       // Min Speed of Servo
  uint32_t max_pwm_value;   // Max PWM Value, use https://1drv.ms/x/s!AjwkVZL4JwUqicdM2-osK8LSTu_taQ?e=FnrZeL
  uint32_t min_pwm_value;   // Min PWM Value, use https://1drv.ms/x/s!AjwkVZL4JwUqicdM2-osK8LSTu_taQ?e=FnrZeL
  uint32_t pwm_frequency;
  uint32_t pwm_clock;
  uint32_t value;
  uint32_t load;
} motor;

typedef struct{
    float time;
    float linear_accel_x;
    float linear_accel_y;
    float linear_accel_z;
    float linear_velocity_x;
    float linear_velocity_y;
    float linear_velocity_z;
    float linear_position_x;
    float linear_position_y;
    float linear_position_z;

    float angular_accel_x;
    float angular_accel_y;
    float angular_accel_z;
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    float angular_position_x;
    float angular_position_y;
    float angular_position_z;

} state;




// State Space
float trapezoidal_integration(float a, float b, float f_a, float f_b);
void pwm_init();
void set_servo_angle(servo* servo_to_modify, int angle);
void servo_init(servo* servo_to_initialize);
void set_motor_speed(motor* motor_0, motor* motor_1, int32_t speed_0, int32_t speed_1);
void motor_init(motor* motor_to_initialize);
void go_forward(motor* motor_0, motor* motor_1, int32_t motor_speed);
void go_backward(motor* motor_0, motor* motor_1, int32_t motor_speed);
uint32_t state_space_init(void);
static void state_space_method(void *p);

// Sensors
static void sensor_read_method(void *p );
uint32_t sensor_read_init(void);


