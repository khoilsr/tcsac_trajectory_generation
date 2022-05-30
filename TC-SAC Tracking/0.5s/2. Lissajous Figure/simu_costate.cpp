#include "simu_costate.h"

class COSTATE_EQ
{
    struct ODE_PARAMS ode_params;

public:
    COSTATE_EQ(struct ODE_PARAMS u) : ode_params(u) {}
    void operator() ( const state_type &rho, state_type &drhodt, const double t  )
    {
        arma::mat Df_dx(rho.size(),rho.size(),arma::fill::zeros);

        // ----- interpolation --------------------------------------------------------------------------
        state_type u(ode_params.m);
        state_type x(rho.size());
        for (int iter=0;iter < ode_params.m;iter++)
        {
            u[iter] = ode_params.s[iter](t); //input;
        }

        for (int iter=0;iter < ode_params.n;iter++)
        {
            x[iter] = ode_params.s[iter+ode_params.m](t); //state;
        }

        // ----- derivatives --------------------------------------------------------------------------
        df_dx(x, u, Df_dx);

        if (ode_params.trajectory_tracking)
        {
            ode_params.x_d=reference_trajectory(t);
        }
        dynamic_obstacle(t,ode_params);

        arma::vec Dl1_dx(x.size(), 1, arma::fill::zeros);
        dl1_dx(x, ode_params.x_d, ode_params, Dl1_dx, u);

        // ----- ode --------------------------------------------------------------------------
        // write rho to data type arma::vec
        arma::vec rho_vec(rho.size());
        for ( int iter = 0; iter < rho.size(); iter++ )
        {
            rho_vec(iter) = rho[iter];
        }

        // costate ode
        rho_vec =   -Dl1_dx - Df_dx.t()*rho_vec;

        // save costate ode
        for(int iter=0; iter<rho.size(); iter++)
        {
            drhodt[iter] = rho_vec(iter);
        }
    }
};

void simu_costate(state_type &rho, state_type &time_grid_backwards,
                  ODE_PARAMS ode_params, SAC_PARAMS sac_params, arma::mat &RHO)
{
    // ---- DEFINITION & INITIALIZATION ------------------------------------------------------
    using namespace boost::numeric::odeint;

    // define stepper to be used
    runge_kutta4< state_type > stepper;

    // iterator for saving data in correct order of time
    int iterator = time_grid_backwards.size()-1;

    // initialize costate equation
    class COSTATE_EQ costate_eq(ode_params);

    // ------- ODE SOLVER ----------------------------------------------------------------------
    // solve ode one step at a time backwards
    for( int timer=0; timer < time_grid_backwards.size(); timer++ )
    {
        // save values of costate in correct order of time
        for (int iter=0; iter < rho.size(); iter++)
        {
            RHO(iter, iterator) = rho[iter];
        }

        // solve ode one step
        stepper.do_step( costate_eq, rho, time_grid_backwards[timer], -sac_params.t_sample);

        // decrement iterator
        iterator--;
    }

}
