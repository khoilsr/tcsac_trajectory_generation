#ifndef TC_SAC_H
#define TC_SAC_H

#include <iostream>
#include <limits>
#include <math.h>


#include "definitions.h"
#include "interpolate_state.h"
#include "interpolate_torque.h"

// ode equations
#include "simu_state.h"
#include "simu_costate.h"
#include "simu_backODEs.h"
#include "reference_trajectory.h"


// cost functions
#include "finalcost.h"

// bryson ho functions
#include "calc_nu.h"
#include "calc_delta_u.h"

 extern void tc_sac(DIM dim, SAC_PARAMS sac_params, BH_PARAMS bh_params, COST_PARAMS cost_params,INPUT_LIMIT input_limit, ODE_PARAMS& ode_params, state_type& x_d, state_type& x, arma::mat& u1, arma::mat& X, arma::mat& X_star,
                    state_type& time_grid, state_type& time_grid_backwards , bool& input,arma::mat& X_BH,arma::mat& u_bh,arma::mat& u_star, bool SAC,int q);
#endif // TC_SAC_H
