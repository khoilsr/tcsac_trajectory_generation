#include "tc_sac.h"

void tc_sac( DIM dim, SAC_PARAMS sac_params, BH_PARAMS bh_params, COST_PARAMS cost_params, INPUT_LIMIT input_limit,
             ODE_PARAMS& ode_params, state_type& x_d, state_type& x, arma::mat& u1, arma::mat& X, arma::mat& X_star,
             state_type& time_grid, state_type& time_grid_backwards , bool& input,arma::mat& X_BH,arma::mat& u_bh,arma::mat& u_star, bool SAC, int q)
{
//// --------------------------------------------------------------------------------------------------
//// -------------- VARIABLES -------------------------------------------------------------------------
//// --------------------------------------------------------------------------------------------------

    // variables ODE
    input=false;
    state_type rho;
    arma::mat RHO(dim.n, time_grid.size(), arma::fill::zeros);
    arma::cube R = arma::zeros<arma::cube>(dim.n, q, time_grid.size());
    arma::mat Ipsipsi(q, q);
    arma::vec Ijpsi(q);
    arma::mat Ijj;

    // variable bryson ho
    arma::vec nu(dim.n);
    arma::mat delta_u(dim.m, u1.n_cols, arma::fill::zeros);

    // variables for line search
    int k = 0;
    double Tm;
    double lambda;
    double t0;
    double tf;
    int index;
    arma::mat u2_star_Tm(dim.m,1);
    int index_start;
    int index_end;
    arma::mat u_insert(arma::size(u1));
    dynamic_obstacle(time_grid[0],ode_params);

//// --------------------------------------------------------------------------------------------------
//// --------------------------------------------------------------------------------------------------
//// -------------- TC-SAC ----------------------------------------------------------------------------
//// --------------------------------------------------------------------------------------------------
//// --------------------------------------------------------------------------------------------------

////_____BRYSON HO PART________________________________________________________________________________
//std::cout<< "BHo" <<std::endl;
    // interpolation
    // /*

    interpolate_torque(u1, time_grid, ode_params);
    // FORWARD INTEGRATION STATE

    simu_state(x, time_grid, ode_params, sac_params, X);
    arma::vec psi_old(dim.n);
    double J_old;
    for (int iter=0; iter<dim.n; iter++)
    {
        psi_old(iter) = X(iter,X.n_cols-1) - x_d[iter];
    }
    J_old = arma::as_scalar(X(dim.n,X.n_cols-1));

    // PROCESSING DATA
    // x_init = X(1,1:4);
    x.clear();
    for (int iter=0; iter<dim.n; iter++)
    {
        x.push_back(X(iter,0));
    }
    x.push_back(0);

    // interpolation
    interpolate_state(X, time_grid, ode_params);

    if (ode_params.trajectory_tracking)
    {
        x_d=reference_trajectory(time_grid[X.n_cols-1]);
    }

    // BACKWARD INTEGRATION
    state_type states(2*(dim.n + pow(dim.n,2))+1, 0);
    for (int iter = 0; iter < dim.n; iter++)
    {
        states[iter] =  2*cost_params.P1[iter]*(X(iter, X.n_cols-1) - x_d[iter]) ;
        states[dim.n + iter*(dim.n + 1)] = 1;
    }


   simu_backODEs(states, time_grid_backwards, ode_params, sac_params, RHO, R, Ipsipsi, Ijpsi, Ijj,q);
//   if (time_grid[0] > 8.9){
//   std::cout << "time: " <<  time_grid[0] << std::endl;
//   std::cout << "RHO  " <<  RHO << std::endl;
//}
    // CALCULATING NU

    calc_nu( Ipsipsi, Ijpsi, X(arma::span(0,dim.n-1), X.n_cols-1), x_d, bh_params, nu ,q);

//    arma::vec delta_psi_pre(dim.n);
//    delta_psi_pre = - Ijpsi - Ipsipsi*nu;
//    std::cout << "delta psi predict: [" << delta_psi_pre[0] << ", " << delta_psi_pre[1]  << "]" << std::endl;

//    double delta_J_pre;
//    delta_J_pre = arma::as_scalar(-(Ijj - Ijpsi.t()*inv(Ipsipsi)*Ijpsi) + Ijpsi.t()*inv(Ipsipsi)*delta_psi_pre);
//    std::cout << "delta_J predict: " << delta_J_pre << std::endl;

    // UPDATING u1

    calc_delta_u(X, RHO, R, nu, bh_params, delta_u , u1, ode_params);

    u1 = u1 + delta_u;


    for (int iter = 0; iter < dim.m; iter++)
    {
        for (int iter2 = 0; iter2 < 1 + floor(sac_params.T_pre/sac_params.t_sample); iter2++)
        {

            if (u1(iter,iter2) > input_limit.max[iter])
            {u1(iter,iter2)=input_limit.max[iter];}
            else if(u1(iter,iter2) < input_limit.min[iter])
            {u1(iter,iter2)=input_limit.min[iter];}
        }
    }

    // interpolation
    interpolate_torque(u1, time_grid, ode_params);
    simu_state(x, time_grid, ode_params, sac_params, X);

//    arma::vec psi_new(dim.n);
//    double J_new;
//    for (int iter=0; iter<dim.n; iter++)
//    {
//        psi_new(iter) = X(iter,X.n_cols-1) - x_d[iter];
//    }
//    J_new = arma::as_scalar(X(dim.n,X.n_cols-1));
////    std::cout << "psi_new: [" << psi_new(0) << ", " << psi_new(1) << ", " << psi_new(2) << ", " << psi_new(3) << "]" << std::endl;
////    std::cout << "J_new: " << J_new << std::endl;
//    //actual change of final state and cost
//    std::cout << "delta psi actual: [" << psi_new(0)-psi_old(0) << ", " << psi_new(1)-psi_old(1) << ", " << psi_new(2)-psi_old(2) << ", " << psi_new(3)-psi_old(3) << "]" << std::endl;
//    std::cout << "delta_J actual: " << J_new - J_old << std::endl;


    interpolate_state(X, time_grid, ode_params);

    x.clear();
    for(int iter=0; iter < dim.n+1; iter++)
    {
        x.push_back(X(iter, X.n_cols-1));
    }


    finalcost(x, x_d, cost_params);
    X(dim.n, X.n_cols-1) = x[x.size()-1];

    X_BH=X;
    u_bh=u1;
      // */
////_____SAC PART___________________________________________________________________________________
    if (SAC)
    {// FORWARD INTEGRATION
    //std::cout<< "SAC" <<std::endl;
    //if (X(dim.n, X.n_cols-1)>0.1)
    {
//    interpolate_torque(u1, time_grid, ode_params);
//    simu_state(x, time_grid, ode_params, sac_params, X);
//    interpolate_state(X, time_grid, ode_params);

    // PROCESSING DATA
    // calculate total cost and store in last entry of X



    // x_init = X(1,1:4);
    x.clear();
    for (int iter=0; iter<dim.n; iter++)
    {
        x.push_back(X(iter,0));
    }
    x.push_back(0);
    // BACKWARD INTEGRATION CO-STATE
    rho.clear();



    for (int iter = 0; iter < dim.n; iter++)
    {
        rho.push_back( 2*cost_params.P1[iter]*(X(iter, X.n_cols-1) - x_d[iter]) );
    }


    simu_costate(rho, time_grid_backwards, ode_params, sac_params, RHO);
    // OPTIMAL CONTROL ACTION
    // index = sum(round(Tx*SAC_params.rnd_factor)<=round(Tm*SAC_params.rnd_factor));
    Tm = time_grid[0] + sac_params.Tm_sample;
    index = 0;
    for ( int iter = 0; iter < time_grid.size(); iter++ )
    {
        if( round( time_grid[iter] * sac_params.rnd_factor ) <= round( Tm*sac_params.rnd_factor ) )
        {
            index += 1;
        }
    }

    arma::mat Df_du(dim.n, dim.m, arma::fill::zeros);
    state_type x_index;
    for ( int iter = 0; iter < ode_params.n; iter++ )
    {
        x_index.push_back(X(iter,index-1));
    }

    state_type u_index;
    for ( int iter = 0; iter < ode_params.m; iter++ )
    {
        u_index.push_back(u1(iter,index-1));
    }



    df_du(x_index, u_index, Df_du, ode_params);

    //u2_star_Tm = inv(Df_du.t()*RHO.col(index-1)*RHO.col(index-1).t()*Df_du + sac_params.S.t())*(Df_du.t()*RHO.col(index-1)*RHO.col(index-1).t()*Df_du*u1.col(index-1) + Df_du.t()*RHO.col(index-1)*sac_params.w_alpha );


    u2_star_Tm = inv(Df_du.t()*RHO.col(index-1)*RHO.col(index-1).t()*Df_du + sac_params.S.t())*(Df_du.t()*RHO.col(index-1)*RHO.col(index-1).t()*Df_du*u1.col(index-1) + Df_du.t()*RHO.col(index-1)*sac_params.w_alpha*X(X.n_rows-1, X.n_cols-1) );
    //u2_star_Tm = solve(Df_du.t()*RHO.col(index-1)*RHO.col(index-1).t()*Df_du + sac_params.S.t(), Df_du.t()*RHO.col(index-1)*RHO.col(index-1).t()*Df_du*u1.col(index-1) + Df_du.t()*RHO.col(index-1)*sac_params.w_alpha*X(X.n_rows-1, X.n_cols-1) );

    for(int iter = 0; iter < dim.m; iter++)
    {
        if (u2_star_Tm(iter) > input_limit.max[iter])
        {u2_star_Tm(iter)=input_limit.max[iter];}
        else if(u2_star_Tm(iter) < input_limit.min[iter])
        {u2_star_Tm(iter)=input_limit.min[iter];}
    }

    // CONTROL DURATION

    k = 0;
    // X_star = X
    X_star = X;
    // set cost for optimal control computed by SAC to infinity
    X_star(X_star.n_rows-1, X_star.n_cols-1) = std::numeric_limits<double>::infinity();

    while (X_star(X_star.n_rows-1, X_star.n_cols-1) >= X(X.n_rows-1, X.n_cols-1) && k < sac_params.k_max)
    {
        // lambda = SAC_params.omega^k*SAC_params.delta_t;
        lambda = pow(sac_params.omega, k)*sac_params.delta_t;

        // t0 = double(round((Tm - lambda/2)*SAC_params.rnd_factor)/SAC_params.rnd_factor);
        t0 = (double) round( (Tm - lambda/2)*sac_params.rnd_factor )/sac_params.rnd_factor;
        if (t0 < time_grid[0])
        {
            t0 = time_grid[0];
        }

        // tf = double(round((Tm + lambda/2)*SAC_params.rnd_factor)/SAC_params.rnd_factor);
        tf = (double) round( (Tm + lambda/2)*sac_params.rnd_factor )/sac_params.rnd_factor;

        // index_start = sum(round(Tx*SAC_params.rnd_factor)<=round(t0*SAC_params.rnd_factor));
        // index_end = sum(round(Tx*SAC_params.rnd_factor)<=round(tf*SAC_params.rnd_factor));
        index_start = 0;
        index_end = 0;
        for ( int iter = 0; iter < time_grid.size(); iter++ )
        {
            if ( round( time_grid[iter]*sac_params.rnd_factor ) <= round( t0*sac_params.rnd_factor ) )
            {
                index_start += 1;
            }
            if ( round( time_grid[iter]*sac_params.rnd_factor ) <= round( tf*sac_params.rnd_factor ) )
            {
                index_end += 1;
            }
        }

        // u_insert(:, 1:(index_start-1)) = u1_new(:, 1:(index_start-1));
        u_insert.cols(0, index_start-2) = u1.cols(0, index_start-2);

        // u_insert(:, index_start:index_end) = u2_star_Tm*ones(1,index_end-index_start+1);
        u_insert.cols(index_start-1, index_end-1) = u2_star_Tm*( arma::ones<arma::mat>(1,index_end-index_start+1) );

        // u_insert(:, (index_end+1):end) = u1_new(:, (index_end+1):end);
        u_insert.cols(index_end, u_insert.n_cols-1) = u1.cols(index_end, u1.n_cols-1);

        // INTEGRATION
        // interpolation
        interpolate_torque(u_insert, time_grid, ode_params);

        simu_state(x, time_grid, ode_params, sac_params, X_star);

        // CALCULATING COST J1
        // calculate total cost and store in last entry of X
        x.clear();
        for(int iter=0; iter < dim.n+1; iter++)
        {
            x.push_back(X_star(iter, X_star.n_cols-1));
        }

        finalcost(x, x_d, cost_params);
        X_star(dim.n, X_star.n_cols-1) = x[x.size()-1];
        // x_init = X(1,1:4);
        x.clear();
        for (int iter=0; iter<dim.n; iter++)
        {
            x.push_back(X(iter,0));
        }
        x.push_back(0);
        u_star=u_insert;

        if ( X_star(dim.n, X_star.n_cols-1) < X(dim.n, X.n_cols-1) )
        {
            u1 = u_insert;
            X = X_star;
            std::cout << "SAC takes over at " << time_grid[0] << std::endl;
            input=true;
        }
        k++;

    }
    }
    }
    //std::cout << "X: " << X << std::endl;
//    if (ode_params.trajectory_tracking)
//    {
//        x_d=reference_trajectory(time_grid[0]);
//    }
//    for (int iter = 0; iter < ode_params.x_d.size(); iter++)
//    {
//        cost += 0.005*cost_params.Q1[iter] * pow(X(iter, 0) - ode_params.x_d[iter], 2);
//    }
}
