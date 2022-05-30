#include "simu_backODEs.h"

class backODE_EQ
{
    struct ODE_PARAMS ode_params;

public:
    backODE_EQ(struct ODE_PARAMS u) : ode_params(u) {}
    void operator() ( const state_type &state, state_type &deriv, const double t  )
    {
        // ----- Variables --------------------------------------------------------------------------
        state_type u(ode_params.m);
        state_type x(ode_params.n);

        arma::mat Df_dx(ode_params.n,ode_params.n,arma::fill::zeros);
        arma::vec Dl1_dx(ode_params.n, 1, arma::fill::zeros);
        arma::mat Df_du(ode_params.n, ode_params.m, arma::fill::zeros);
        arma::mat Dl1_du(1, ode_params.m, arma::fill::zeros);

        arma::vec rho_vec(ode_params.n);
        arma::vec rho_vec_new(ode_params.n);
        arma::mat r_mat(ode_params.n, ode_params.n);
        arma::vec r_vec(ode_params.n*ode_params.n);
        arma::vec ipsipsi_vec(ode_params.n*ode_params.n);
        arma::vec ijpsi_vec (ode_params.n);
        arma::mat ijj_scalar(1,1);

        // ----- data -----------------------------------------------------------------
        for ( int iter1 = 0; iter1 < ode_params.n; iter1++ )
        {
            rho_vec(iter1) = state[iter1];
            for( int iter2 = 0; iter2 < ode_params.n; iter2++ )
            {
                r_mat(iter2, iter1) = state[ode_params.n + iter1*(r_mat.n_rows) + iter2];
                //std::cout<<state[ode_params.n + iter1*(r_mat.n_rows) + iter2]<<std::endl;
            }
        }

        //std::cout<<r_mat<<std::endl;

        // ----- interpolation --------------------------------------------------------------------------

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

        dl1_dx(x, ode_params.x_d, ode_params, Dl1_dx, u);
        df_du(x, u, Df_du, ode_params);
        dl1_du(x, u, Dl1_du, ode_params);

        // ----- ode --------------------------------------------------------------------------
        // costate ode
        rho_vec_new =  -Dl1_dx - Df_dx.t()*rho_vec;

        // R ode
        r_vec = vectorise(-Df_dx.t()*r_mat);
//std::cout<<Df_dx<<std::endl;
        // Ipsipsi eq
        ipsipsi_vec = vectorise( -r_mat.t() * Df_du * ode_params.W * Df_du.t() * r_mat );
//std::cout<<rho_vec.t()*Df_du<<std::endl;
//std::cout<<Df_du<<std::endl;
//std::cout<<Dl1_du<<std::endl;
//std::cout<<ode_params.W<<std::endl;
//std::cout<<r_mat<<std::endl;
        // Ijpsi eq
        ijpsi_vec = -( (rho_vec.t()*Df_du + Dl1_du) * ode_params.W * Df_du.t() * r_mat ).t();
//std::cout<<ijpsi_vec<<std::endl;
//std::cout<<"break"<<std::endl;

        // Ijj eq
        ijj_scalar = -(rho_vec.t()*Df_du + Dl1_du) * ode_params.W * (Df_du.t()*rho_vec + Dl1_du.t());


        // clear deriv and attach new ode values
        deriv.clear();
        deriv.insert(deriv.end(), rho_vec_new.begin(), rho_vec_new.end());
        deriv.insert(deriv.end(), r_vec.begin(), r_vec.end());
        deriv.insert(deriv.end(), ipsipsi_vec.begin(), ipsipsi_vec.end());
        deriv.insert(deriv.end(), ijpsi_vec.begin(), ijpsi_vec.end());
        deriv.insert(deriv.end(), ijj_scalar.begin(), ijj_scalar.end());
    }
};

void simu_backODEs(state_type &states, state_type time_grid_backwards, ODE_PARAMS ode_params, SAC_PARAMS sac_params,
                   arma::mat &RHO, arma::cube &R, arma::mat &Ipsipsi, arma::vec &Ijpsi, arma::mat &Ijj, int q )
{
    using namespace boost::numeric::odeint;

    ////Ignore desired states


    //define stepper to be used
    runge_kutta4< state_type > stepper;

    // iterator for saving data of costate/R
    int iterator = time_grid_backwards.size()-1;

    // initialize ode
    class backODE_EQ backode_eq(ode_params);

    // integrate one step at a time
    // vector for integrations is structured as follows:
    // entries 0        through (n-1)       : rho
    // entries n        through n+(n^2-1)   : R
    // entries n+n^2    through n+2*n^2-1   : Ipsipsi
    // entries n+2*n^2  through n+2*n^2+n-1 : Ijpsi
    for( int timer=0 ; timer < time_grid_backwards.size() ; timer++ )
    {

        // save values of costate and R
        for (int iter1=0; iter1 < q; iter1++)
        {
            RHO(iter1, iterator) = states[iter1];
            for( int iter2 = 0; iter2 < ode_params.n; iter2++ )
            {
                R(iter2, iter1, iterator) = states[ode_params.n + iter1*(ode_params.n) + iter2];
            }
        }

        // integration
        stepper.do_step( backode_eq, states, time_grid_backwards[timer], -sac_params.t_sample);

        iterator--;
    }


//    for(uint i=0;i<states.size();i++){
//        std::cout<< states[i]<<std::endl;
//    }
    //std::cout<<"break"<<std::endl;
    // save values of Integrals Ijpsi and Ipsipsi
    for(int row=0; row<q; row++)
    {
        Ijpsi(row) = states[ode_params.n+2*pow(ode_params.n, 2)+row];

        for(int col=0; col<q; col++)
        {
            Ipsipsi(row,col) = states[ode_params.n+pow(ode_params.n, 2) + row*ode_params.n+col];
        }
    }
    Ijj = states[ode_params.n+2*pow(ode_params.n,2)+ode_params.n];

}
