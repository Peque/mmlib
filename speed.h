#ifndef __SPEED_H
#define __SPEED_H

#include "math.h"

#include "mmlib/common.h"
#include "mmlib/control.h"
#include "mmlib/move.h"
#include "mmlib/path.h"

#include "config.h"
#include "setup.h"

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

struct turn_parameters get_turn_parameters(enum movement turn_type);
float get_max_force(void);
void set_max_force(float value);
float get_linear_acceleration(void);
float get_linear_deceleration(void);
float get_max_linear_speed(void);
void set_max_linear_speed(float value);
void kinematic_configuration(float force, bool run);
float get_move_turn_before(enum movement move);
float get_move_turn_after(enum movement move);
float get_move_turn_linear_speed(enum movement turn_type, float force);

#endif /* __SPEED_H */
