#include "calc_delta_u.h"

void calc_delta_u(arma::mat X, arma::mat RHO, arma::cube R, arma::vec nu, BH_PARAMS bh_params, arma::mat &delta_u, arma::mat u1, ODE_PARAMS ode_params)
{
    state_type x(X.n_rows);
    state_type u(u1.n_rows);


    arma::mat Df_du(X.n_rows-1, delta_u.n_rows, arma::fill::zeros);
    arma::mat Dl1_du(1, delta_u.n_rows, arma::fill::zeros);

    for(int iter=0; iter<X.n_cols; iter++)
    {
        // state at current time step iter
        for(int iter2=0; iter2<x.size(); iter2++)
        {
            x[iter2] = X(iter2, iter);
        }
        for(int iter2=0; iter2<u.size(); iter2++)
        {
            u[iter2] = u1(iter2, iter);
        }

        // Df_du at time step iter
        df_du(x, u, Df_du, ode_params);
        dl1_du(x, u, Dl1_du, ode_params);
        //std::cout<< nu <<std::endl;
        // calculate delta_u as given by Bryson/Ho
        // inverse of matrix W is already calculated in main function before start of tc-sac
        delta_u.col(iter) = - bh_params.W * ( Dl1_du + (RHO.col(iter) + R.slice(iter)*nu).t() * Df_du ).t();
    }
}
