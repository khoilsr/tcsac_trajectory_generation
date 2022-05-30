#ifndef INTERPOLATE_TORQUE_H
#define INTERPOLATE_TORQUE_H

#include "spline.h"

#include "definitions.h"

extern void interpolate_torque(arma::mat u, state_type time_grid, ODE_PARAMS &ode_params);

#endif // INTERPOLATE_TORQUE_H
