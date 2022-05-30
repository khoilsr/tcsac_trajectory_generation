#ifndef DF_DU_H
#define DF_DU_H

#include <math.h>

#include "definitions.h"

extern void df_du(state_type& x, state_type& u,arma::mat& Jf, ODE_PARAMS ode_params);

#endif // DF_DU_H
