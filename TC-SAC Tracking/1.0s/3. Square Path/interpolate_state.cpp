#include "interpolate_state.h"

void interpolate_state(arma::mat X, state_type time_grid, ODE_PARAMS& ode_params)
{
    std::vector<std::vector<double>> val_x (X.n_cols);
    for ( int i = 0 ; i < X.n_cols; i++ )
    {
        val_x[i].resize(ode_params.n);
    }

    for (int iter=0;iter < ode_params.n;iter++)
    {
        val_x[iter].resize(X.n_cols);

        for(int iter2=0; iter2 < X.n_cols; iter2++)
        {
            val_x[iter] [iter2] = X(iter,iter2);
        }
    }

        for (int iter=ode_params.m;iter < ode_params.m+ode_params.n;iter++)
        {
            ode_params.s[iter].set_points(time_grid, val_x[iter-ode_params.m]);
        }

}
