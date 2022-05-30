#include "dl1_du.h"

void dl1_du(state_type x, state_type u,arma::mat &Jl1, ODE_PARAMS ode_params)
{

//    for (int iter=0;iter < ode_params.m;iter++)
//    {

//            if ( u[iter] >= 0.9*ode_params.input_limit.max[iter])
//            {
//                Jl1[iter] += 2*ode_params.c_max*( u[iter] - 0.9*ode_params.input_limit.max[iter]);
//            }

//            if ( u[iter] <= 0.9* ode_params.input_limit.min[iter])
//            {
//                Jl1[iter] += 2*ode_params.c_min*( u[iter] - 0.9* ode_params.input_limit.min[iter]);
//            }
//    }
    for (int iter = 0; iter < u.size(); iter++)
    {
        Jl1[iter] += 2*ode_params.Q_control[iter] * u[iter];
    }
}
