#ifndef INTERPOLATE_STATE_H
#define INTERPOLATE_STATE_H

#include "spline.h"

#include "definitions.h"

extern void interpolate_state(arma::mat X, state_type time_grid, ODE_PARAMS& ode_params);

#endif // INTERPOLATE_STATE_H
