#include "control.h"

static volatile float target_linear_speed;
static volatile float ideal_linear_speed;
static volatile float ideal_angular_speed;

static volatile float linear_error;
static volatile float angular_error;
static volatile float last_linear_error;
static volatile float last_angular_error;
static volatile float last_ideal_angular_speed;

static volatile float voltage_left;
static volatile float voltage_right;
static volatile int32_t pwm_left;
static volatile int32_t pwm_right;

static volatile bool collision_detected_signal;
static volatile bool motor_control_enabled_signal;
static volatile bool side_sensors_close_control_enabled;
static volatile bool side_sensors_far_control_enabled;
static volatile bool front_sensors_control_enabled;
static volatile bool diagonal_sensors_control_enabled;
static volatile float side_sensors_integral;
static volatile float front_sensors_integral;
static volatile float diagonal_sensors_integral;

/**
 * @brief Convert a given voltage to its corresponding motor PWM duty.
 *
 * This function reads the current motor driver input voltage first to adjust
 * the PWM output accordingly. Useful when powering the motor driver directly
 * from a battery or to compensate for possible voltage drops in DC-DC
 * converters.
 *
 * @param[in] voltage Voltage to convert to its corresponding PWM duty.
 */
static int32_t voltage_to_motor_pwm(float voltage)
{
	return voltage / get_motor_driver_input_voltage() * DRIVER_PWM_PERIOD;
}

/**
 * @brief Enable or disable the side sensors close control.
 */
void side_sensors_close_control(bool value)
{
	side_sensors_close_control_enabled = value;
}

/**
 * @brief Enable or disable the side sensors far control.
 */
void side_sensors_far_control(bool value)
{
	side_sensors_far_control_enabled = value;
}

/**
 * @brief Enable or disable the diagonal control.
 */
void diagonal_sensors_control(bool value)
{
	diagonal_sensors_control_enabled = value;
}

/**
 * @brief Enable or disable the front sensors control.
 */
void front_sensors_control(bool value)
{
	front_sensors_control_enabled = value;
}

/**
 * @brief Disable sensors control.
 */
void disable_walls_control(void)
{
	side_sensors_close_control(false);
	side_sensors_far_control(false);
	front_sensors_control(false);
}

/**
 * @brief Set collision detected signal.
 *
 * It also automatically disables the motor control.
 */
static void set_collision_detected(void)
{
	collision_detected_signal = true;
	motor_control_enabled_signal = false;
}

/**
 * @brief Returns true if a collision was detected.
 */
bool collision_detected(void)
{
	return collision_detected_signal;
}

/**
 * @brief Reset the collision detection signal.
 *
 * This will also reset the PWM saturation counters, used for collision
 * detection.
 */
void reset_collision_detection(void)
{
	collision_detected_signal = false;
	reset_motor_driver_saturation();
}

/**
 * @brief Reset control error variables.
 */
void reset_control_errors(void)
{
	side_sensors_integral = 0;
	front_sensors_integral = 0;
	diagonal_sensors_integral = 0;
	linear_error = 0;
	angular_error = 0;
	last_linear_error = 0;
	last_angular_error = 0;
}

/**
 * @brief Reset control speed variables.
 */
void reset_control_speed(void)
{
	target_linear_speed = 0.;
	ideal_linear_speed = 0.;
	ideal_angular_speed = 0.;
}

/**
 * @brief Reset all control variables.
 *
 * In particular:
 *
 * - Reset control errors.
 * - Reset control speed.
 * - Reset collision detection.
 */
void reset_control_all(void)
{
	reset_control_errors();
	reset_control_speed();
	reset_collision_detection();
}

/**
 * @brief Enable the motor control.
 *
 * This means the motor control function will be executed the PWM output will be
 * generated.
 */
void enable_motor_control(void)
{
	motor_control_enabled_signal = true;
}

/**
 * @brief Disable the motor control.
 *
 * This means the motor control function will not be executed and no PWM output
 * will be generated.
 */
void disable_motor_control(void)
{
	motor_control_enabled_signal = false;
}

/**
 * @brief Reset motion to an iddle state.
 *
 * - Disable motor control.
 * - Disable walls control.
 * - Turn the motor driver off.
 * - Reset control state.
 */
void reset_motion(void)
{
	disable_motor_control();
	disable_walls_control();
	drive_off();
	reset_control_all();
}

float get_side_sensors_integral(void)
{
	return side_sensors_integral;
}

float get_front_sensors_integral(void)
{
	return front_sensors_integral;
}

float get_diagonal_sensors_integral(void)
{
	return diagonal_sensors_integral;
}

/**
 * @brief Return the current voltage for the left motor.
 */
float get_left_motor_voltage(void)
{
	return voltage_left;
}

/**
 * @brief Return the current voltage for the right motor.
 */
float get_right_motor_voltage(void)
{
	return voltage_right;
}

/**
 * @brief Return the current PWM duty for the left motor.
 */
int32_t get_left_pwm(void)
{
	return pwm_left;
}

/**
 * @brief Return the current PWM duty for the right motor.
 */
int32_t get_right_pwm(void)
{
	return pwm_right;
}

/**
 * @brief Return the current target linear speed in meters per second.
 */
float get_target_linear_speed(void)
{
	return target_linear_speed;
}

/**
 * @brief Return the current ideal linear speed in meters per second.
 */
float get_ideal_linear_speed(void)
{
	return ideal_linear_speed;
}

/**
 * @brief Return the current ideal angular speed in radians per second.
 */
float get_ideal_angular_speed(void)
{
	return ideal_angular_speed;
}

/**
 * @brief Return the current measured linear speed in meters per second.
 */
float get_measured_linear_speed(void)
{
	return (get_encoder_left_speed() + get_encoder_right_speed()) / 2.;
}

/**
 * @brief Return the current measured angular speed in radians per second.
 */
float get_measured_angular_speed(void)
{
	return -get_gyro_z_radps();
}

/**
 * @brief Set target linear speed in meters per second.
 */
void set_target_linear_speed(float speed)
{
	target_linear_speed = speed;
}

/**
 * @brief Set ideal angular speed in radians per second.
 */
void set_ideal_angular_speed(float speed)
{
	ideal_angular_speed = speed;
}

static float calculate_next_ideal_linear_speed(void)
{
	float next = ideal_linear_speed;

	if (next < target_linear_speed) {
		next += get_linear_acceleration() / SYSTICK_FREQUENCY_HZ;
		if (next > target_linear_speed)
			next = target_linear_speed;
	} else if (next > target_linear_speed) {
		next -= get_linear_deceleration() / SYSTICK_FREQUENCY_HZ;
		if (next < target_linear_speed)
			next = target_linear_speed;
	}
	return next;
}

static float calculate_next_force(float next_ideal_linear_speed)
{
	float vdiff = next_ideal_linear_speed - ideal_linear_speed;
	float accel = vdiff * SYSTICK_FREQUENCY_HZ;
	return accel * MOUSE_MASS;
}

/**
 * @brief Update ideal linear speed according to the defined speed profile.
 *
 * Current ideal speed is increased or decreased according to the target speed
 * and the defined maximum acceleration and deceleration.
 */
void update_ideal_linear_speed(void)
{
	if (ideal_linear_speed < target_linear_speed) {
		ideal_linear_speed +=
		    get_linear_acceleration() / SYSTICK_FREQUENCY_HZ;
		if (ideal_linear_speed > target_linear_speed)
			ideal_linear_speed = target_linear_speed;
	} else if (ideal_linear_speed > target_linear_speed) {
		ideal_linear_speed -=
		    get_linear_deceleration() / SYSTICK_FREQUENCY_HZ;
		if (ideal_linear_speed < target_linear_speed)
			ideal_linear_speed = target_linear_speed;
	}
}

static float get_speed_direction(void)
{
	if (target_linear_speed > 0.)
		return 1.;
	if (target_linear_speed < 0.)
		return -1.;
	return 0.;
}

/**
 * @brief Execute the robot motor control.
 *
 * Set the motors power to try to follow a defined speed profile.
 *
 * This function also implements collision detection by checking PWM output
 * saturation. If collision is detected it sets the `collision_detected_signal`
 * variable to `true`.
 */
void motor_control(void)
{
	float feedback_linear;
	float feedback_angular;
	float feedforward_left;
	float feedforward_right;
	float side_sensors_feedback = 0.;
	float front_sensors_feedback = 0.;
	float diagonal_sensors_feedback = 0.;
	struct control_constants control;
	float next_ideal_linear_speed;
	float next_force;
	float angular_acceleration;
	float angular_acceleration_force;

	if (!motor_control_enabled_signal)
		return;

	if (side_sensors_close_control_enabled) {
		side_sensors_feedback += get_side_sensors_close_error();
		side_sensors_integral += side_sensors_feedback;
	}

	if (side_sensors_far_control_enabled) {
		side_sensors_feedback += get_side_sensors_far_error();
		side_sensors_integral += side_sensors_feedback;
	}

	if (front_sensors_control_enabled) {
		front_sensors_feedback = get_front_sensors_error();
		front_sensors_integral += front_sensors_feedback;
	}

	if (diagonal_sensors_control_enabled) {
		diagonal_sensors_feedback = get_diagonal_sensors_error();
		diagonal_sensors_integral += diagonal_sensors_feedback;
	}

	linear_error += ideal_linear_speed - get_measured_linear_speed();
	angular_error += ideal_angular_speed - get_measured_angular_speed();

	control = get_control_constants();

	feedback_linear =
	    control.kp_linear * linear_error +
	    control.kd_linear * (linear_error - last_linear_error);
	feedback_angular =
	    control.kp_angular * angular_error +
	    control.kd_angular * (angular_error - last_angular_error) +
	    control.kp_angular_side * side_sensors_feedback +
	    control.kp_angular_front * front_sensors_feedback +
	    control.kp_angular_diagonal * diagonal_sensors_feedback +
	    control.ki_angular_side * side_sensors_integral +
	    control.ki_angular_front * front_sensors_integral +
	    control.ki_angular_diagonal * diagonal_sensors_integral;

	next_ideal_linear_speed = calculate_next_ideal_linear_speed();
	next_force = calculate_next_force(next_ideal_linear_speed);

	feedforward_left = FEEDFORWARD_SPEED_LEFT_0 * get_speed_direction() +
			   FEEDFORWARD_SPEED_LEFT_1 * next_ideal_linear_speed +
			   FEEDFORWARD_FORCE_LEFT * next_force / 2.;
	feedforward_right =
	    FEEDFORWARD_SPEED_RIGHT_0 * get_speed_direction() +
	    FEEDFORWARD_SPEED_RIGHT_1 * next_ideal_linear_speed +
	    FEEDFORWARD_FORCE_RIGHT * next_force / 2.;

	angular_acceleration = (ideal_angular_speed - last_ideal_angular_speed) *
			       SYSTICK_FREQUENCY_HZ;
	angular_acceleration_force =
	    (angular_acceleration * MOUSE_MOMENT_OF_INERTIA /
	     MOUSE_WHEELS_SEPARATION);

	feedforward_left += FEEDFORWARD_SPEED_LEFT_1 * ideal_angular_speed * MOUSE_WHEELS_SEPARATION / 2.;
	feedforward_right -= FEEDFORWARD_SPEED_RIGHT_1 * ideal_angular_speed * MOUSE_WHEELS_SEPARATION / 2.;

	feedforward_left += FEEDFORWARD_FORCE_LEFT * angular_acceleration_force;
	feedforward_right -= FEEDFORWARD_FORCE_RIGHT * angular_acceleration_force;

        if (ideal_angular_speed > 0) {
                feedforward_left += FEEDFORWARD_TURN_RIGHT_LEFT;
                feedforward_right += FEEDFORWARD_TURN_RIGHT_RIGHT;
        } else if (ideal_angular_speed < 0) {
                feedforward_left += FEEDFORWARD_TURN_LEFT_LEFT;
                feedforward_right += FEEDFORWARD_TURN_LEFT_RIGHT;
        }

	voltage_left = feedforward_left + feedback_linear + feedback_angular;
	voltage_right = feedforward_right + feedback_linear - feedback_angular;
	pwm_left = voltage_to_motor_pwm(voltage_left);
	pwm_right = voltage_to_motor_pwm(voltage_right);

	power_left(pwm_left);
	power_right(pwm_right);

	last_linear_error = linear_error;
	last_angular_error = angular_error;
        last_ideal_angular_speed = ideal_angular_speed;

	ideal_linear_speed = next_ideal_linear_speed;

	if (motor_driver_saturation() >
	    MAX_MOTOR_DRIVER_SATURATION_PERIOD * SYSTICK_FREQUENCY_HZ)
		set_collision_detected();
}
