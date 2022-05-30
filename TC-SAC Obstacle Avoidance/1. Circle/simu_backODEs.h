#ifndef SIMU_BACKODES_H
#define SIMU_BACKODES_H

#include <math.h>

#include <boost/numeric/odeint.hpp>

#include "definitions.h"
#include "correct_angle.h"
#include "df_dx.h"
#include "df_du.h"
#include "dl1_dx.h"
#include "dl1_du.h"
#include "reference_trajectory.h"
#include "dynamic_obstacle.h"


extern void simu_backODEs(state_type &states, state_type time_grid_backwards, ODE_PARAMS ode_params, SAC_PARAMS sac_params,
                          arma::mat &RHO, arma::cube &R, arma::mat &Ipsipsi, arma::vec &Ijpsi, arma::mat &Ijj,int q);

#endif // SIMU_BACKODES_H
