#ifndef CALC_DELTA_U_H
#define CALC_DELTA_U_H

#include <math.h>

#include "definitions.h"
#include "df_du.h"
#include "dl1_du.h"

extern void calc_delta_u(arma::mat X, arma::mat RHO, arma::cube R, arma::vec nu, BH_PARAMS bh_params, arma::mat &delta_u, arma::mat u1, ODE_PARAMS ode_params);


#endif // CALC_DELTA_U_H
