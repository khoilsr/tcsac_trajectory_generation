#ifndef CALC_NU_H
#define CALC_NU_H

#include <math.h>


#include "definitions.h"

extern void calc_nu(arma::mat &Ipsipsi, arma::vec &Ijpsi, arma::mat x_f, state_type &x_d, BH_PARAMS bh_params, arma::vec &nu, int q );

#endif // CALC_NU_H
