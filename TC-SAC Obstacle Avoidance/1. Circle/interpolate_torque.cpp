#include "interpolate_torque.h"

void interpolate_torque(arma::mat u1, state_type time_grid, ODE_PARAMS &ode_params)
{
    std::vector<std::vector<double>> val_u (u1.n_cols);
    for ( int i = 0 ; i < u1.n_cols; i++ )
    {
        val_u[i].resize(ode_params.m);
    }

    for (int iter=0;iter < ode_params.m;iter++)
    {
        val_u[iter].resize(u1.n_cols);

        for(int iter2=0; iter2 < u1.n_cols; iter2++)
        {
            val_u[iter] [iter2] = u1(iter,iter2);
        }
    }

        for (int iter=0;iter < ode_params.m;iter++)
        {
            ode_params.s[iter].set_points(time_grid, val_u[iter]);
        }


}
