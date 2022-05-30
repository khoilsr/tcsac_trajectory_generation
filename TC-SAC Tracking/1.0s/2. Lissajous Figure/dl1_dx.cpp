#include "dl1_dx.h"

using namespace arma;
using namespace std;

void dl1_dx(state_type& x, state_type &x_d, ODE_PARAMS ode_params, vec& Jl1, state_type &u)
{
    // tracking cost
    for( int iter = 0; iter < ode_params.Q.size(); iter++ )
    {
        Jl1(iter) = 2*ode_params.Q[iter]*(x[iter] - x_d[iter]);
    }

//    for( int iter = 0; iter < ode_params.theta_max.size(); iter++ )
//    {
//            if ( x[iter] >= ode_params.theta_max[iter] - ode_params.theta_tol[iter] )
//            {
//                Jl1(iter) += 2*ode_params.c_max*( x[iter] - (ode_params.theta_max[iter] - ode_params.theta_tol[iter]));
//            }

//            if ( x[iter] <= ode_params.theta_min[iter] + ode_params.theta_tol[iter] )
//            {
//                Jl1(iter) += 2*ode_params.c_min*( x[iter] - (ode_params.theta_min[iter] + ode_params.theta_tol[iter]));
//            }
//    }


    for (int iter = 0; iter < ode_params.obstacle_r.size(); iter++)
    {
            if ( sqrt(pow(x[0]-ode_params.obstacle_x[iter],2)+pow(x[1]-ode_params.obstacle_y[iter],2)) <= ode_params.obstacle_r[iter])
            {
                Jl1(0) = -2*ode_params.obstacle_avoid[0]*(x[0]-ode_params.obstacle_x[iter])*(ode_params.obstacle_r[iter]-sqrt(pow(x[0]-ode_params.obstacle_x[iter],2)+pow(x[1]-ode_params.obstacle_y[iter],2)));
                Jl1(1) = -2*ode_params.obstacle_avoid[0]*(x[1]-ode_params.obstacle_y[iter])*(ode_params.obstacle_r[iter]-sqrt(pow(x[0]-ode_params.obstacle_x[iter],2)+pow(x[1]-ode_params.obstacle_y[iter],2)));
            }
    }


}


