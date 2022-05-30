#include "calc_nu.h"

void calc_nu( arma::mat &Ipsipsi, arma::vec &Ijpsi, arma::mat x_f, state_type &x_d, BH_PARAMS bh_params, arma::vec &nu , int q)
{
    arma::vec x_d_vec(q);
    arma::vec x_f_select(q);

    for(int iter=0; iter < 2; iter++)
    {
        x_d_vec(iter) = x_d[iter];
        x_f_select(iter)=x_f(iter);
    }

//    for(int iter=3; iter < 7; iter++)
//    {
//        x_d_vec(iter-2) = x_d[iter];
//        x_f_select(iter-2)=x_f(iter);
//    }
//    x_d_vec(2) = x_d[4];
//    x_f_select(2)=x_f(4);
    //std::cout <<std::endl;

    //x_d_vec(2)=atan((x_f[1]-x_d[1])/(x_f[0]-x_d[0]));
    // calculate nu defined by Bryson/Ho

    //nu = -pinv(Ipsipsi)*(bh_params.tol_psi*(x_f - x_d_vec) - Ijpsi);
//    std::cout<<"dPsi:"<<bh_params.tol_psi*(x_f_select - x_d_vec)<<std::endl;
//    std::cout<<"Ijpsi:"<<Ijpsi<<std::endl;
//    std::cout<<"break:"<<std::endl;
    //std::cout<<bh_params.tol_psi<<std::endl;
//std::cout<<x_f<<std::endl;
//std::cout<<x_d_vec<<std::endl;
//std::cout<<bh_params.tol_psi*(x_f - x_d_vec)<<std::endl;
//std::cout<<Ijpsi<<std::endl;

    //nu = arma::solve(Ipsipsi, bh_params.tol_psi*(x_f_select - x_d_vec) - Ijpsi);
    //nu = -inv(Ipsipsi)*(bh_params.tol_psi*(x_f - x_d_vec) + Ijpsi);
//    std::cout<<Ipsipsi<<std::endl;
//    std::cout<<bh_params.tol_psi*(x_f_select -x_d_vec)<<std::endl;
//    std::cout<<(Ijpsi)<<std::endl;

    nu = inv(Ipsipsi)*(bh_params.tol_psi*(x_f_select - x_d_vec) - Ijpsi);
    //nu = arma::solve(Ipsipsi, bh_params.tol_psi*(x_f_select - x_d_vec) - Ijpsi);


}
