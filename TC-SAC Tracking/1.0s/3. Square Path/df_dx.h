#ifndef DF_DX_H
#define DF_DX_H

#include <math.h>

#include "definitions.h"
#include "correct_angle.h"
#include "correct_alpha.h"


extern void df_dx(state_type& x, state_type& u, arma::mat& Jf);

#endif // DF_DX_H
