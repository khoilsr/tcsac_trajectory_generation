#ifndef SIMU_COSTATE_H
#define SIMU_COSTATE_H

#include <math.h>

#include <boost/numeric/odeint.hpp>

#include "definitions.h"
#include "df_dx.h"
#include "dl1_dx.h"
#include "spline.h"
#include "correct_angle.h"
#include "reference_trajectory.h"
#include "dynamic_obstacle.h"


extern void simu_costate(state_type &rho, state_type &time_grid_backwards, ODE_PARAMS ode_params, SAC_PARAMS sac_params, arma::mat &RHO);

#endif // SIMU_COSTATE_H
