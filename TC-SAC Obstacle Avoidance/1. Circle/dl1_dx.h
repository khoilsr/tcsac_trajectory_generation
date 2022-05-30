#ifndef DL1_DX_H
#define DL1_DX_H

#include <math.h>
#include <cmath>

#include "definitions.h"

extern void dl1_dx(state_type& x, state_type& x_d, ODE_PARAMS ode_params, arma::vec& Jl1, state_type &u);

#endif // DL1_DX_H
