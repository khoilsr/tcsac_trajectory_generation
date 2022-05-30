#include "df_du.h"

using namespace arma;

void df_du(state_type& x, state_type& u, mat& Jf, ODE_PARAMS ode_params)
{
    // User can define Df_du here; only non-zero elements need to be defined

    const double m=1200, theta=2400, l_f=1.35, l_r=1.45,
    mF0 = 621.43, mR0 = 578.57, r=0.3, cw=0.31, rho= 1.293,
    A=2.0, muyf = 0.8, muyr = 1.1, g = 9.81;

    double C_1 = 0.5;
    double C_2 = 600;
    double C_3 = 3000;
    double C_4 = 50;
    double C_5 = -1;


    double  x0=x[0],
            x1=x[1],
            x2=x[2],
            x3=x[3],
            x4=x[4],
            x5=x[5],
            x6=x[6];

       Jf(3,1) = (cos(x4)/r+cos(x4-x6)/r)/m;
       Jf(4,1) = -(sin(x4)/r+sin(x4-x6)/r)/(m*x3);
       Jf(5,1) = (l_f*sin(x6))/(r*theta);
       Jf(6,0) = 1.0;//ode_params.t_sample;


}
