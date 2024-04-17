#include "state_space.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define STATESPACESTACKSIZE        1500         // Stack size in words
#define SENSORREADSTACKSIZE        128         // Stack size in words

// Global Variables
state robot_state;
state robot_state_previous;
struct bno055_euler_float_t xyz_angles; // converted data to euler angles
struct bno055_accel_float_t xyz_accel;
// State Space Task
float desired_angle = -96.08;
float angle_error = 0.0;
float angle_error_sum = 0.0;
float motor_input = 0.0;
float Kp = 200.0;
float Ki = 0.0;
float Kd = 0.0;
motor left_wheel;
motor right_wheel;

float trapezoidal_integration(float a, float b, float f_a, float f_b){
    float value;
    value = (f_a + f_b) / 2.0 * (b-a);
    return value;
}

void pwm_init(){
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    //Configure PF1,PF2,PF3 Pins as PWM
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    //Configure PF1,PF2,PF3 Pins as PWM
    GPIOPinConfigure(GPIO_PC4_M0PWM6);
    GPIOPinConfigure(GPIO_PC5_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
}

void set_servo_angle(servo* servo_to_modify, int angle){
    uint32_t pulse_width;
    uint32_t value;

    value = servo_to_modify->min_pwm_value+(angle-servo_to_modify->min_angle)*((servo_to_modify->max_pwm_value - servo_to_modify->min_pwm_value)/(servo_to_modify->max_angle-servo_to_modify->min_angle));
    pulse_width = value * servo_to_modify->load / 65535;


//    UARTprintf("Value: %d, ", servo_to_modify->load);
//    UARTprintf("Value: %d\n", pulse_width);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, pulse_width);
}


void servo_init(servo* servo_to_initialize){
    servo_to_initialize->pwm_clock = SysCtlClockGet() / 64;
    servo_to_initialize->load = (servo_to_initialize->pwm_clock / servo_to_initialize->pwm_frequency) - 1;

    UARTprintf("Load: %d\n", servo_to_initialize->load);
    UARTprintf("Clock: %d\n", servo_to_initialize->pwm_clock);
    UARTprintf("Frequency: %d\n", servo_to_initialize->pwm_frequency);

    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, servo_to_initialize->load);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);

    set_servo_angle(servo_to_initialize, 45.0);

    // Turn on the Output pins
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
    // Enable the PWM generator
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
}

void set_motor_speed(motor* motor_0, motor* motor_1, int32_t speed_0, int32_t speed_1){


    int32_t motor_speed_0;
    int32_t motor_speed_1;

    motor_speed_0 = abs(speed_0);
    motor_speed_1 = abs(speed_1);

    uint32_t pulse_width_0;
    uint32_t pulse_width_1;

    uint32_t value_0;
    uint32_t value_1;


    value_0 = motor_0->min_pwm_value+(motor_speed_0-motor_0->min_speed)*((motor_0->max_pwm_value - motor_0->min_pwm_value)/(motor_0->max_speed-motor_0->min_speed));
    value_1 = motor_1->min_pwm_value+(motor_speed_1-motor_1->min_speed)*((motor_1->max_pwm_value - motor_1->min_pwm_value)/(motor_1->max_speed-motor_1->min_speed));

//    value_0 = value_0;
//    value_1 = value_1;

    if(speed_0 >= 0){
        PWMPulseWidthSet(motor_0->pwm_base_0, motor_0->pwm_out_0, value_0);
    }
    else{
        PWMPulseWidthSet(motor_0->pwm_base_0, motor_0->pwm_out_1, value_0);
    }

    if(speed_1 >= 0){
        PWMPulseWidthSet(motor_1->pwm_base_1, motor_1->pwm_out_0, value_1);
    }
    else{
        PWMPulseWidthSet(motor_1->pwm_base_1, motor_1->pwm_out_1, value_1);
    }
}

void motor_init(motor* motor_to_initialize){
    motor_to_initialize->pwm_clock = SysCtlClockGet() / 64;
    motor_to_initialize->load = (motor_to_initialize->pwm_clock / motor_to_initialize->pwm_frequency) - 1;
    motor_to_initialize->max_pwm_value = motor_to_initialize->load;
    motor_to_initialize->max_speed = motor_to_initialize->load;
    motor_to_initialize->min_pwm_value = motor_to_initialize->load / 2;
    motor_to_initialize->min_speed = motor_to_initialize->load / 2;

    UARTprintf("Load: %d\n", motor_to_initialize->load);
    UARTprintf("Clock: %d\n", motor_to_initialize->pwm_clock);
    UARTprintf("Frequency: %d\n", motor_to_initialize->pwm_frequency);

    PWMGenPeriodSet(motor_to_initialize->pwm_base_0, motor_to_initialize->pwm_gen_0, motor_to_initialize->load);
    PWMGenConfigure(motor_to_initialize->pwm_base_0, motor_to_initialize->pwm_gen_0, PWM_GEN_MODE_DOWN);

    PWMGenPeriodSet(motor_to_initialize->pwm_base_1, motor_to_initialize->pwm_gen_1, motor_to_initialize->load);
    PWMGenConfigure(motor_to_initialize->pwm_base_1, motor_to_initialize->pwm_gen_1, PWM_GEN_MODE_DOWN);

//    set_motor_speed(motor_to_initialize, -255, -255);

    // Turn on the Output pins
    PWMOutputState(motor_to_initialize->pwm_base_0, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);

    // Enable the PWM generator
    PWMGenEnable(motor_to_initialize->pwm_base_0, motor_to_initialize->pwm_gen_0);
    PWMGenEnable(motor_to_initialize->pwm_base_1, motor_to_initialize->pwm_gen_1);
}




//void go_forward(motor* motor_0, motor* motor_1, int32_t motor_speed){
//    motor_speed = constrain(motor_speed, -255, 255);
//    set_motor_speed(motor_0, 0, motor_speed);
//    set_motor_speed(motor_1, 0, motor_speed);
//
//}
//void go_backward(motor* motor_0, motor* motor_1, int32_t motor_speed){
//    motor_speed = constrain(motor_speed, -255, 255);
//    set_motor_speed(motor_0, motor_speed, 0);
//    set_motor_speed(motor_1, motor_speed, 0);
//
//}

void print_state(char* axis, float val){
  char msg_buf[30]; // message
  sprintf(msg_buf, "%s = %f\n", axis, val);
  UARTprintf("%s", msg_buf);
}
static void state_space_method(void *p){
    const TickType_t output_delay = 1 / portTICK_PERIOD_MS;


    int local_time = 0;
    robot_state.time = local_time;

    robot_state.linear_accel_x = xyz_accel.x;
    robot_state.linear_accel_y = xyz_accel.y;
    robot_state.linear_accel_z = xyz_accel.z;
    robot_state.linear_velocity_x = 0.0;
    robot_state.linear_velocity_y = 0.0;
    robot_state.linear_velocity_z = 0.0;
    robot_state.linear_position_x = 0.0;
    robot_state.linear_position_y = 0.0;
    robot_state.linear_position_z = 0.0;

    while(1){
        local_time += 1;
        robot_state.time = local_time;

        robot_state.linear_accel_x = xyz_accel.x;
        robot_state.linear_accel_y = xyz_accel.y;
        robot_state.linear_accel_z = xyz_accel.z;

        robot_state.angular_position_x = xyz_angles.h * 180.0 / 3.14159;
        robot_state.angular_position_y = xyz_angles.r * 180.0 / 3.14159;
        robot_state.angular_position_z = xyz_angles.p * 180.0 / 3.14159;

        if (local_time == 0){
            robot_state_previous = robot_state;
        }


        // X
        robot_state.linear_velocity_x += trapezoidal_integration(robot_state_previous.time, robot_state.time, robot_state_previous.linear_accel_x, robot_state.linear_accel_x);
        robot_state.linear_position_x += trapezoidal_integration(robot_state_previous.time, robot_state.time, robot_state_previous.linear_velocity_x, robot_state.linear_velocity_x);

        // Y
        robot_state.linear_velocity_y += trapezoidal_integration(robot_state_previous.time, robot_state.time, robot_state_previous.linear_accel_y, robot_state.linear_accel_y);
        robot_state.linear_position_y += trapezoidal_integration(robot_state_previous.time, robot_state.time, robot_state_previous.linear_velocity_y, robot_state.linear_velocity_y);

        // Z
        robot_state.linear_velocity_z += trapezoidal_integration(robot_state_previous.time, robot_state.time, robot_state_previous.linear_accel_z, robot_state.linear_accel_z);
        robot_state.linear_position_z += trapezoidal_integration(robot_state_previous.time, robot_state.time, robot_state_previous.linear_velocity_z, robot_state.linear_velocity_z);


        angle_error = robot_state.angular_position_x - desired_angle;
        angle_error_sum  += angle_error;
        //angle_error_sum = constrain(angle_error_sum, -180, 180);

        motor_input = Kp * (angle_error) + Ki * angle_error_sum * (robot_state.time - robot_state_previous.time) + Kd*(robot_state.angular_position_x - robot_state_previous.angular_position_x)/(robot_state.time - robot_state_previous.time);
        set_motor_speed(&left_wheel, &right_wheel, motor_input, motor_input);
        print_state("Angle: ", robot_state.angular_position_x);
        print_state("Motor Speed: ", motor_input);

//        if((int)robot_state.angular_position_x < -90){
//            go_backward(&left_wheel, &right_wheel, motor_input);
//        }
//        else{
//            go_forward(&left_wheel, &right_wheel, motor_input);
//        }

        robot_state_previous = robot_state;
        vTaskDelay(output_delay);
    }
}


uint32_t state_space_init(void){
    pwm_init();

    ms_delay(10);



//    left_wheel.min_speed = 0;
////    left_wheel.max_speed = 255;
//
////    left_wheel.max_pwm_value = 100000;
//    left_wheel.min_pwm_value = 0;
    left_wheel.pwm_frequency = 20000;
    left_wheel.pwm_base_0 = PWM1_BASE;
    left_wheel.pwm_base_1 = PWM1_BASE;
    left_wheel.pwm_out_0 = PWM_OUT_6;
    left_wheel.pwm_out_1 = PWM_OUT_7;
    left_wheel.pwm_gen_0 = PWM_GEN_3;
    left_wheel.pwm_gen_1 = PWM_GEN_3;

    right_wheel.min_speed = 0;
//    right_wheel.max_speed = 255;
//    right_wheel.max_pwm_value = 100000;
    right_wheel.min_pwm_value = 0;
    right_wheel.pwm_frequency = 20000;
    right_wheel.pwm_base_0 = PWM0_BASE;
    right_wheel.pwm_base_1 = PWM0_BASE;
    right_wheel.pwm_out_0 = PWM_OUT_6;
    right_wheel.pwm_out_1 = PWM_OUT_7;
    right_wheel.pwm_gen_0 = PWM_GEN_3;
    right_wheel.pwm_gen_1 = PWM_GEN_3;

    motor_init(&left_wheel);
    motor_init(&right_wheel);

    //
    // Create the LED task.
    //
    if(xTaskCreate(state_space_method, (const portCHAR *)"STATE SPACE", STATESPACESTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_STATE_SPACE_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}

// Sensor Read Task
static void sensor_read_method(void *p ){
    const TickType_t output_delay = 50 / portTICK_PERIOD_MS;
    while(1){
        xyz_accel = get_linear_acceleration();
        xyz_angles = get_abs_position();
        vTaskDelay(output_delay);
    }
}


uint32_t sensor_read_init(void){
    // Initialize the I2C
    ConfigureI2C();

    ms_delay(10);
    // Initialize the sensor
    init_imu();

    // Set the sensor operation mode, note in this mode the sensor should be calibrated first
    set_imu_mode(BNO055_OPERATION_MODE_NDOF);

    int32_t calibrated = 0;
    Calibration cal;

    while(calibrated == 0){

      cal = calibrate_imu();

      UARTprintf("Calibration: ");
      UARTprintf("Gyro: %d\t", cal.gyro);
      UARTprintf("Acc: %d\t", cal.accl);
      UARTprintf("Mag: %d\t", cal.magn);
      UARTprintf("Sys: %d\n", cal.syst);

      if(cal.syst == 3){
        calibrated = 1;
      }

      ms_delay(10);
    }

    if(xTaskCreate(sensor_read_method, (const portCHAR *)"sensor", SENSORREADSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SENSOR_READ_TASK, NULL) != pdTRUE){
        return(1);
    }
    return 0;

}

