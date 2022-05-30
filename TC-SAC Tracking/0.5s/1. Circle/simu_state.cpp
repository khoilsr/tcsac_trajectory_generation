#include "simu_state.h"


class STATE_EQ
{
    struct ODE_PARAMS ode_params;

    const double m=1200, theta=2400, l_f=1.35, l_r=1.45,
    mF0 = 621.43, mR0 = 578.57, r=0.3, cw=0.31, rho= 1.293,
    A=2.0, muyf = 0.8, muyr = 1.1, g = 9.81, k=4;

    double FzF = mF0*g, FzR = mR0*g;

    double alphaF, alphaR, FxF, FxR, FyF, FyR;

public:
    STATE_EQ(struct ODE_PARAMS u) : ode_params(u) {}
    void operator() ( const state_type &x, state_type &dxdt, const double t )
    {

//// STEP 2.1
////##### interpolation ########
        arma::mat u(ode_params.m,1);
        for (int iter=0;iter < ode_params.m;iter++)
        {
            u(iter) = ode_params.s[iter](t); //input;
        }

        //std::cout << u << std::endl;
////##########################################

//// STEP 2.2
////##### definition of ode #######

double steer = x[6];


//            if (x[6] > 45*M_PI/180)
//            {steer= 45*M_PI/180;}
//            else if (x[6] < -45*M_PI/180)
//            {steer= -45*M_PI/180;}
//            else
//            {steer= x[6];}



        alphaF = atan2((x[3] * sin(x[4]) + l_f * x[5]), (x[3] * cos(x[4]))) - steer;
        alphaR = atan2((x[3] * sin(x[4]) - l_r * x[5]), (x[3] * cos(x[4])));

        FxF =u(1)/0.3; //convert from torque to force
        FxR =u(1)/0.3; //convert from torque to force

        FyF = 2 * tire_model(alphaF, FzF/2, muyf);
        FyR = 2 * tire_model(alphaR, FzR/2, muyr);





        dxdt[0]= x[3]*cos(x[4]+x[2]);
        dxdt[1]= x[3]*sin(x[4]+x[2]);
        dxdt[2]= x[5];
        dxdt[3]= (FxF * cos(x[4] - steer) + FxR * cos(x[4]) + FyF * sin(x[4] - steer) + FyR * sin(x[4]) -x[3]/fabs(x[3])*0.5*cw*rho*A*pow(x[3],2))/m;
        dxdt[4]= ( - FxF * sin(x[4] - steer) - FxR * sin(x[4]) + FyF * cos(x[4] - steer) + FyR * cos(x[4]) - m * x[3] * x[5]) / (m * x[3]);
        dxdt[5]= (FxF * l_f * sin(steer) + FyF * l_f * cos(steer) - FyR * l_r) / theta;
        dxdt[6]= u(0) - k *x[6];



//        if (x[6] >= 45*M_PI/180 && u(0) > 0)
//        {dxdt[6]= 0;}
//        else if (x[6] <= -45*M_PI/180 && u(0) < 0)
//        {dxdt[6]= 0;}



        ////##########################################
//// STEP 2.3
////##### definition of cost function ######
        // cost functional
        dxdt[ode_params.n] = 0;
        if (ode_params.trajectory_tracking)
        {
            ode_params.x_d=reference_trajectory(t);
        }
        //x_d tracking
        for (int iter = 0; iter < ode_params.x_d.size(); iter++)
        {
            dxdt[ode_params.n] += ode_params.Q[iter] * pow(x[iter] - ode_params.x_d[iter], 2);
        }

        //Steering angle limitation
//        for (int iter = 0; iter < ode_params.x_d.size(); iter++)
//        {
//                if ( x[iter] >= ode_params.theta_max[iter] - ode_params.theta_tol[iter] )
//                {
//                    dxdt[ode_params.n] += ode_params.c_max*pow( x[iter] - (ode_params.theta_max[iter] - ode_params.theta_tol[iter]), 2);
//                }

//                if ( x[iter] <= ode_params.theta_min[iter] + ode_params.theta_tol[iter] )
//                {
//                    dxdt[ode_params.n] += ode_params.c_min*pow( x[iter] - (ode_params.theta_min[iter] + ode_params.theta_tol[iter]), 2);
//                }
//        }

//        //Input Cost
//        for (int iter=0;iter < ode_params.m;iter++)
//        {

//                if ( u[iter] >= 0.9*ode_params.input_limit.max[iter])
//                {
//                    dxdt[ode_params.n] += ode_params.c_max*pow( u[iter] -  0.9*ode_params.input_limit.max[iter], 2);
//                }

//                if ( u[iter] <= 0.9* ode_params.input_limit.min[iter])
//                {
//                    dxdt[ode_params.n] += ode_params.c_min*pow( u[iter] - 0.9* ode_params.input_limit.min[iter],2);
//                }
//        }

        for (int iter = 0; iter < u.size(); iter++)
        {
            dxdt[ode_params.n] += ode_params.Q_control[iter] * pow(u[iter], 2);
        }

        dynamic_obstacle(t,ode_params);

        //std::cout << "Obst_x:" << ode_params.obstacle_x[0] << std::endl;
        //std::cout << t << std::endl;

        //Obstacle Avoidance
        for (int iter = 0; iter < ode_params.obstacle_r.size(); iter++)
        {
                if ( sqrt(pow(x[0]-ode_params.obstacle_x[iter],2)+pow(x[1]-ode_params.obstacle_y[iter],2)) <= ode_params.obstacle_r[iter])
                {
                    //std::cout << ode_params.obstacle_avoid[0]*pow(ode_params.obstacle_r[iter] - sqrt(pow((x[0]-ode_params.obstacle_x[iter]),2)+pow((x[1]-ode_params.obstacle_y[iter]),2)),2) << std::endl;

                    dxdt[ode_params.n] += ode_params.obstacle_avoid[0]*pow(ode_params.obstacle_r[iter] - sqrt(pow((x[0]-ode_params.obstacle_x[iter]),2)+pow((x[1]-ode_params.obstacle_y[iter]),2)),2);
                }
        }

    }
};

struct push_back_state_and_time
{
    std::vector< state_type >& m_states;
    std::vector< double >& m_times;

    push_back_state_and_time( std::vector< state_type > &states , std::vector< double > &times )
    : m_states( states ) , m_times( times ) { }

    void operator()( const state_type &x , double t )
    {
        m_states.push_back( x );
        m_times.push_back( t );
    }
};

void simu_state(state_type &x, state_type &time_grid, ODE_PARAMS &ode_params, SAC_PARAMS sac_params, arma::mat &X)
{
    // iterator for saving data
    int iterator = 0;    

    using namespace boost::numeric::odeint;

    // define stepper to be used
    runge_kutta4< state_type > stepper;

    // initialize ode equation
    class STATE_EQ state_eq(ode_params);

    // solve ode step by step
    for( int timer=0 ; timer < time_grid.size(); timer++ )
    {
        // save states
        for(int iter=0; iter<x.size(); iter++)
        {
            X(iter,iterator) = x[iter];
        }


        // integrate one step
        stepper.do_step( state_eq, x, time_grid[timer], sac_params.t_sample);


        x[2] = correct_angle(x[2]);
        x[4] = correct_angle(x[4]);


//        x[6] = correct_angle(x[6]);
        // increment iterator
        iterator++;
    }
   // std::cout << X << std::endl;

}




