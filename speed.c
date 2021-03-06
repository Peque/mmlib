#include "speed.h"

/**
 * Speed module static variables.
 *
 * - Maximum force applied on the tires.
 * - Maximum linear speed.
 */
static volatile float max_force;
static volatile float max_linear_speed;

/**
 * Parameters that define a turn.
 *
 * - Meters to travel in straight line before turning
 * - Meters to travel in straight line after turning
 * - Curve minimum radius
 * - Duration, in meters, of the angular acceleration phase
 * - Duration, in meters, of the constant angular velocity phase
 * - Sign of the turn (left or right)
 */
struct turn_parameters {
	float before;
	float after;
	float radius;
	float transition;
	float arc;
	int sign;
};

/**
 * @brief Calculate the maximum search linear speed.
 *
 * This speed is calculated so that the search speed in long straight lines
 * is always constant.
 *
 * @param[in] force Maximum force to apply while searching.
 *
 * @return The calculated search linear speed.
 */
static float _calculate_search_linear_speed(float force)
{
	float turn_velocity;
	float break_margin;

	turn_velocity = get_move_turn_linear_speed(MOVE_LEFT, force);
	break_margin = get_move_turn_before(MOVE_LEFT);
	break_margin -= turn_velocity * SEARCH_REACTION_TIME;
	return sqrt(turn_velocity * turn_velocity +
		    2 * get_linear_deceleration() * break_margin);
}

/**
 * @brief Configure force and search/run mode.
 *
 * - Higher force results in higher accelerations.
 * - Search mode limits the maximum linear speed for a smoother and more stable
 *   search.
 *
 * @param[in] force Maximum force to apply on the tires.
 * @param[in] run Whether to set speed variables for the run phase or not.
 */
void kinematic_configuration(float force, bool run)
{
	max_force = force;
	if (run)
		max_linear_speed = get_linear_speed_limit();
	else
		max_linear_speed = _calculate_search_linear_speed(force);
}

// clang-format off
struct turn_parameters turns[] = {
    [MOVE_LEFT] = {0.01700, 0.01700, 0.04921, 0.06042, 0.00037, -1},
    [MOVE_RIGHT] = {0.01700, 0.01700, 0.04921, 0.06042, 0.00037, 1},
    [MOVE_LEFT_90] = {-0.06272, -0.06272, 0.13000, 0.06042, 0.12728, -1},
    [MOVE_RIGHT_90] = {-0.06272, -0.06272, 0.13000, 0.06042, 0.12728, 1},
    [MOVE_LEFT_180] = {-0.04500, -0.04500, 0.08882, 0.06042, 0.20211, -1},
    [MOVE_RIGHT_180] = {-0.04500, -0.04500, 0.08882, 0.06042, 0.20211, 1},
    [MOVE_LEFT_TO_45] = {-0.06374, 0.06354, 0.10000, 0.06042, 0.00161, -1},
    [MOVE_RIGHT_TO_45] = {-0.06374, 0.06354, 0.10000, 0.06042, 0.00161, 1},
    [MOVE_LEFT_FROM_45] = {0.06354, -0.06374, 0.10000, 0.06042, 0.00161, -1},
    [MOVE_RIGHT_FROM_45] = {0.06354, -0.06374, 0.10000, 0.06042, 0.00161, 1},
    [MOVE_LEFT_TO_135] = {-0.03813, 0.03642, 0.08000, 0.06042, 0.11157, -1},
    [MOVE_RIGHT_TO_135] = {-0.03813, 0.03642, 0.08000, 0.06042, 0.11157, 1},
    [MOVE_LEFT_FROM_135] = {0.03642, -0.03813, 0.08000, 0.06042, 0.11157, -1},
    [MOVE_RIGHT_FROM_135] = {0.03642, -0.03813, 0.08000, 0.06042, 0.11157, 1},
    [MOVE_LEFT_DIAGONAL] = {0.03888, 0.03888, 0.06500, 0.06042, 0.02518, -1},
    [MOVE_RIGHT_DIAGONAL] = {0.03888, 0.03888, 0.06500, 0.06042, 0.02518, 1},
};
// clang-format on

float get_max_force(void)
{
	return max_force;
}

void set_max_force(float value)
{
	max_force = value;
}

float get_linear_acceleration(void)
{
	return 2 * max_force / MOUSE_MASS;
}

float get_linear_deceleration(void)
{
	return 2 * max_force / MOUSE_MASS;
}

float get_max_linear_speed(void)
{
	return max_linear_speed;
}

void set_max_linear_speed(float value)
{
	max_linear_speed = value;
}

/**
 * @brief Execute a speed turn.
 *
 * @param[in] turn_type Turn type.
 * @param[in] force Maximum force to apply while turning.
 */
void speed_turn(enum movement turn_type, float force)
{
	int32_t start;
	int32_t current;
	float travelled;
	float linear_velocity;
	float angular_velocity;
	float max_angular_velocity;
	float factor;
	struct turn_parameters turn = turns[turn_type];

	linear_velocity = get_move_turn_linear_speed(turn_type, force);
	max_angular_velocity = turn.sign * linear_velocity / turn.radius;

	disable_walls_control();
	start = get_encoder_average_micrometers();
	while (true) {
		current = get_encoder_average_micrometers();
		travelled = (float)(current - start) / MICROMETERS_PER_METER;
		if (travelled >= 2 * turn.transition + turn.arc)
			break;
		angular_velocity = max_angular_velocity;
		if (travelled < turn.transition) {
			factor = travelled / turn.transition;
			angular_velocity *= sin(factor * PI / 2);
		} else if (travelled >= turn.transition + turn.arc) {
			factor = (travelled - turn.arc) / turn.transition;
			angular_velocity *= sin(factor * PI / 2);
		}
		set_ideal_angular_speed(angular_velocity);
	}
	set_ideal_angular_speed(0);
}

/**
 * @brief Get the straight distance that a turn adds before a straight movement.
 *
 * @param[in] turn_type Turn type.
 *
 * @return The added distance.
 */
float get_move_turn_before(enum movement turn_type)
{
	return turns[turn_type].before;
}

/**
 * @brief Get the straight distance that a turn adds after a straight movement.
 *
 * @param[in] turn_type Turn type.
 *
 * @return The added distance.
 */
float get_move_turn_after(enum movement turn_type)
{
	return turns[turn_type].after;
}

/**
 * @brief Get the expected linear speed at which to turn.
 *
 * @param[in] turn_type Turn type.
 * @param[in] force Maximum force to apply while turning.
 *
 * @return The calculated speed.
 */
float get_move_turn_linear_speed(enum movement turn_type, float force)
{
	return sqrt(force * 2 * turns[turn_type].radius / MOUSE_MASS);
}
