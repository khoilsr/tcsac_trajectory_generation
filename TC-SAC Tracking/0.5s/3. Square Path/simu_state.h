#ifndef SIMU_STATE_H
#define SIMU_STATE_H

#include <math.h>

#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <boost/numeric/odeint.hpp>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "definitions.h"
#include "correct_angle.h"
#include "spline.h"
#include "reference_trajectory.h"
#include "dynamic_obstacle.h"
#include "tire_model.h"


#include "interpolate_state.h"

extern void simu_state(state_type &x, state_type &time_grid, ODE_PARAMS &ode_params, SAC_PARAMS sac_params, arma::mat &X);
#endif // SIMU_STATE_H
