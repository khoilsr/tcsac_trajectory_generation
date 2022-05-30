#include "main.h"

int main(int argc, char *argv[])
{

//// STEP 1.1
////####### DEFINE PROBLEM DIMENSIONS ############################################################################
    DIM dim;
        dim.n = 7;
        dim.m = 2;
////##############################################################################################################
    bool input;
    SAC_PARAMS sac_params;

    BH_PARAMS bh_params;
        bh_params.W = arma::zeros<arma::mat>(dim.m,dim.m);

    COST_PARAMS cost_params;
        cost_params.P1.reserve(dim.n);
        cost_params.Q1.reserve(dim.n);

    INPUT_LIMIT input_limit;

    state_type x;
        x.insert(x.end(), dim.n+1, 0);
    state_type x_d;
        x_d.insert(x_d.end(), dim.n, 0);
////###############################################################################################################
////####### USER-SPECIFIC DEFINITIONS #############################################################################
////###############################################################################################################
    // Common PARAMETERS

        // SIMULATION TIME
        double t_stop;
            t_stop = 62.5;

    sac_params.t_sample = 0.02;
        sac_params.rnd_factor = 1/sac_params.t_sample;
    sac_params.Tm_sample = 2*sac_params.t_sample;
        sac_params.delta_t = sac_params.Tm_sample;
    sac_params.T_pre =0.5;

    // SAC PARAMETERS
    bool SAC=true;
    sac_params.k_max = 3;
    sac_params.omega = 0.8;
    sac_params.w_alpha = -pow(10,6);
    sac_params.S = arma::zeros<arma::mat>(dim.m, dim.m);

    sac_params.S(0,0) = pow(10,7);
    sac_params.S(1,1) = pow(10,4);
    // BRYSON/HO PARAMETER
    int q=2;
    bh_params.W(0,0) = pow(10,7);
    bh_params.W(1,1) = pow(10,4);


    bh_params.tol_psi = 0.1;


    // INITIAL STATE

    double v=10;
    x[0] = 0;
    x[1] = 0;
    x[2] = 0*M_PI/180;
    x[3] = v;
    x[4] = 0.1*M_PI/180;
    x[5] = 0.1*M_PI/180;
    x[6] = 0.1*M_PI/180;


    std::ofstream input_file, param_file,sac_result;
    input_file.open("data/result_Square.txt");
    param_file.open("data/params_Square.txt");
    sac_result.open("data/planned_Trajectory.txt");



    // COST PARAMETERS
    cost_params.Q1_control.push_back(0*pow(10,-1)); //1 u_delta
    cost_params.Q1_control.push_back(0*pow(10,-5)); //2 M



        cost_params.Q1.push_back(pow(10,0)); //1 X
        cost_params.Q1.push_back(pow(10,0)); //2 Y
        cost_params.Q1.push_back(0); //3 Psi

        cost_params.Q1.push_back(pow(10,-1)); //4 v
        cost_params.Q1.push_back(0); //5 alpha
        cost_params.Q1.push_back(0); //6 Psi_dot


        cost_params.Q1.push_back(0); //7 delta


        ////##################################################



        cost_params.P1.push_back(0); //1
        cost_params.P1.push_back(0); //2
        cost_params.P1.push_back(0); //3


        cost_params.P1.push_back(0); //4
        cost_params.P1.push_back(pow(10,1)); //5
        cost_params.P1.push_back(0); //6


        cost_params.P1.push_back(0); //7

    // weights for joint limit cost
    cost_params.c_max = pow(10,0);
    cost_params.c_min = pow(10,0);
    // joint limit parameters



    cost_params.theta_max.push_back(pow(10,6)); //0
    cost_params.theta_min.push_back(-pow(10,6));
    cost_params.theta_tol.push_back(0);

    cost_params.theta_max.push_back(pow(10,6)); //1
    cost_params.theta_min.push_back(-pow(10,6));
    cost_params.theta_tol.push_back(0);

    cost_params.theta_max.push_back(pow(10,6)); //2
    cost_params.theta_min.push_back(-pow(10,6));
    cost_params.theta_tol.push_back(0);


    cost_params.theta_max.push_back(pow(10,6)); //3
    cost_params.theta_min.push_back(-pow(10,6));
    cost_params.theta_tol.push_back(0);


    cost_params.theta_max.push_back(pow(10,6)); //4
    cost_params.theta_min.push_back(-pow(10,6));
    cost_params.theta_tol.push_back(0);

    cost_params.theta_max.push_back(pow(10,6)); //5
    cost_params.theta_min.push_back(-pow(10,6));
    cost_params.theta_tol.push_back(0);


    cost_params.theta_max.push_back(pow(10,6)); //6
    cost_params.theta_min.push_back(-pow(10,6));
    cost_params.theta_tol.push_back(0);






    // Obstacle definition

    cost_params.obstacle_avoid.push_back(pow(10,6));


    // INPUT LIMITS

    input_limit.min.insert( input_limit.min.end(), 1, -180*pow(10,0)*M_PI/180 );
    input_limit.max.insert( input_limit.max.end(), 1, 180*pow(10,0)*M_PI/180);

    input_limit.min.insert( input_limit.min.end(), 1, -6*pow(10,2));
    input_limit.max.insert( input_limit.max.end(), 1, 6*pow(10,2));


    // TRACK TRAJECTORY
    cost_params.trajectory_tracking=true; // false for static goal






    if (cost_params.trajectory_tracking)
    {
        state_type x_help;
        x_help.insert(x_help.end(), dim.n, 0);
        x_help=reference_trajectory(0);

        for (int iter=0;iter<2;iter++)
        {
            x[iter]=x_help[iter];
        }
    }


    x_d[0] = 0;
    x_d[1] = 0;
    x_d[2] = 0*M_PI/180;
    x_d[3] = 0;
    x_d[4] = 0*M_PI/180;
    x_d[5] = 0*M_PI/180;
    x_d[6] = 0*M_PI/180;





////###############################################################################################################
////####### NO MORE DEFINTIONS NEEDED #############################################################################
////###############################################################################################################
////###############################################################################################################
////###############################################################################################################
////###############################################################################################################
////###############################################################################################################
////###############################################################################################################
    bh_params.W = inv(bh_params.W);
    arma::mat u1(dim.m, 1 + sac_params.T_pre/sac_params.t_sample, arma::fill::zeros);
    arma::mat u_star(dim.m, 1 + sac_params.T_pre/sac_params.t_sample, arma::fill::zeros);
    arma::mat u_bh(dim.m, 1 + sac_params.T_pre/sac_params.t_sample, arma::fill::zeros);

    // for data saving
    arma::mat X(dim.n+1, 1 + sac_params.T_pre/sac_params.t_sample);
    arma::mat X_BH(dim.n+1, 1 + sac_params.T_pre/sac_params.t_sample);
    arma::mat X_star(dim.n+1, 1 + sac_params.T_pre/sac_params.t_sample);

    // TIME GRID
    state_type time_grid( 1 + sac_params.T_pre/sac_params.t_sample );
        for (uint iter = 0; iter < time_grid.size(); iter++)
        {
            time_grid[iter] += iter * sac_params.t_sample;
        }
    // BACKWARDS TIME GRID
    state_type time_grid_backwards;
        time_grid_backwards.insert(time_grid_backwards.end(), time_grid.rbegin(), time_grid.rend());

    // ODE PARAMETERS
    ODE_PARAMS ode_params;
        ode_params.n = dim.n;
        ode_params.m = dim.m;
        ode_params.x_d.insert(ode_params.x_d.end(), x_d.begin(), x_d.end());
        ode_params.Q.insert(ode_params.Q.end(), cost_params.Q1.begin(), cost_params.Q1.end());
        ode_params.Q_control.insert(ode_params.Q_control.end(), cost_params.Q1_control.begin(), cost_params.Q1_control.end());

        ode_params.W = bh_params.W;
        ode_params.s.resize(dim.n+dim.m);
        ode_params.t_sample = sac_params.t_sample;
        ode_params.input_limit = input_limit;

        std::vector<double> dummy(time_grid.size());
        std::fill(dummy.begin(), dummy.end(), 0);


        for (int iter=0;iter < ode_params.m;iter++)
        {
            ode_params.s[iter].set_points(time_grid, dummy);
        }
        ode_params.trajectory_tracking=cost_params.trajectory_tracking;

        ode_params.c_max = cost_params.c_max;
        ode_params.c_min = cost_params.c_min;

        ode_params.theta_max = cost_params.theta_max;
        ode_params.theta_min = cost_params.theta_min;
        ode_params.theta_tol = cost_params.theta_tol;

        ode_params.obstacle_avoid = cost_params.obstacle_avoid;
        ode_params.obstacle_x = cost_params.obstacle_x;
        ode_params.obstacle_y = cost_params.obstacle_y;
        ode_params.obstacle_r = cost_params.obstacle_r;



    int iter_time = 0;

    double Tm;
//        Tm = 0;

    state_type run_time;
////##### BEGIN TC-SAC ########################################################################################################################
    // open file data.m in folder date to store output values

    // Save Parameters
    param_file << "COMMON PARAMETERS\n\n"

               << "T_sample: " << sac_params.t_sample << "\n"
               << "T_Pre: " << sac_params.T_pre << "\n"

               << "\n\nSAC PARAMETERS\n\n"

               << "SAC: " << SAC << "\n"
               << "sac_params.k_max: " << sac_params.k_max << "\n"
               << "sac_params.omega: " << sac_params.omega << "\n"
               << "sac_params.w_alpha: " << sac_params.w_alpha << "\n"
               << "sac_params.S: \n" << sac_params.S << "\n"

               << "\n\nBRYSON/HO PARAMETERS\n\n"

               << "bh_params.W(0,0): " << bh_params.W(0,0) << "\n"
               << "bh_params.W(1,1): " << bh_params.W(1,1) << "\n"
               << "bh_params.tol_psi: " << bh_params.tol_psi << "\n"

               << "\n\nCOST PARAMETERS\n\n"
               << "cost_params.P1: \n";


    for (int iter=0;iter < ode_params.n;iter++)
    {
    param_file << cost_params.P1[iter] << "\n";
    }
    param_file << "cost_params.Q1_control: \n";
    for (int iter=0;iter < ode_params.m;iter++)
    {
    param_file << cost_params.Q1_control[iter] << "\n";
    }
    param_file << "cost_params.Q1: \n";
    for (int iter=0;iter < ode_params.n;iter++)
    {
    param_file << cost_params.Q1[iter] << "\n";
    }
    param_file << "cost_params.c_max: \n" << cost_params.c_max << "\n"
               << "cost_params.c_min: \n" << cost_params.c_min << "\n\n"
               << "cost_params.theta_max: \n";
    for (int iter=0;iter < ode_params.n;iter++)
    {
    param_file << cost_params.theta_max[iter]<< "\n";
    }
    param_file << "cost_params.theta_min: \n";
    for (int iter=0;iter < ode_params.n;iter++)
    {
    param_file << cost_params.theta_min[iter] << "\n";
    }
    param_file << "cost_params.theta_tol: \n";
    for (int iter=0;iter < ode_params.n;iter++)
    {
    param_file << cost_params.theta_tol[iter] << "\n";
    }
    param_file << "\n\nINPUT LIMITS\n\n"
               << "input_limit.min: \n";

    for (int iter=0;iter < ode_params.m;iter++)
    {
    param_file << input_limit.min[iter]<< "\n";
    }
    param_file << "input_limit.max: \n";
    for (int iter=0;iter < ode_params.m;iter++)
    {
    param_file << input_limit.max[iter]<< "\n";
    }

    param_file << "\n\nObstacles\n\n"
               << "cost_params.obstacle_avoid: \n";
    for (uint iter=0;iter < cost_params.obstacle_avoid.size();iter++)
    {
    param_file << cost_params.obstacle_avoid[iter]<< "\n";
    }
    param_file << "cost_params.obstacle_x: \n";
    for (uint iter=0;iter < cost_params.obstacle_x.size();iter++)
    {
    param_file << cost_params.obstacle_x[iter]<< "\n";
    }
    param_file << "cost_params.obstacle_y: \n";
    for (uint iter=0;iter < cost_params.obstacle_y.size();iter++)
    {
    param_file << cost_params.obstacle_y[iter]<< "\n";
    }
    param_file << "cost_params.obstacle_r: \n";
    for (uint iter=0;iter < cost_params.obstacle_r.size();iter++)
    {
    param_file << cost_params.obstacle_r[iter]<< "\n";
    }

    param_file << "\n\nTRACK TRAJECTORY\n\n"
               << "cost_params.trajectory_tracking: \n" << cost_params.trajectory_tracking << "\n"

               << "\n\nINITIAL STATE\n\n"
               << "x: \n";
    for (uint iter=0;iter < x.size();iter++)
    {
    param_file << x[iter]<< "\n";
    }
    param_file << "\n\nDESIRED STATE\n\n"
               << "x_d: \n";
    for (uint iter=0;iter < x_d.size();iter++)
    {
    param_file << x_d[iter]<< "\n";
    }
    param_file << "\n\nSIMULATION TIME\n\n"
               << "t_stop: \n" << t_stop << "\n";

    param_file.close();


    // define timer
    std::clock_t start;
    std::clock_t all_start;
    double duration;
    // start timer for overall duration
    all_start = std::clock();

    int iterator = 1;
    for( iter_time = 0; iter_time < round(t_stop*sac_params.rnd_factor); iter_time++)
    {

        //if( round(time_grid[0]*sac_params.rnd_factor ) >= round(Tm*sac_params.rnd_factor) )
        { 
            // start timer to measure execution time of one tc-sac step
            start = std::clock();

            Tm = time_grid[0] + sac_params.Tm_sample;
            //for(uint i = 0; i < 10; i++)
            {

                tc_sac( dim, sac_params, bh_params, cost_params, input_limit, ode_params, x_d, x, u1, X, X_star, time_grid, time_grid_backwards,input,X_BH,u_bh,u_star,SAC,q);

                std::cout << "u_delta:" << 180/M_PI*u1(0,1) << std::endl;
                std::cout << "M:" << u1(1,1) << std::endl;


                //std::cout << "Obst_x:" << ode_params.obstacle_x[0] << std::endl;
                std::cout << "Time:" << time_grid[0]<< "s" << std::endl;


            }
            // save execution time of one step of tc-sac
            run_time.push_back( (std::clock() - start) / (double) CLOCKS_PER_SEC );
        }

        // SAVING STATE DATA
        // save current state to data.m in matlab notation

        input_file <<time_grid[0] << " " << X(0,0) << " " << X(1,0) << " " << 180/M_PI*X(2,0) << " " << X(3,0) << " " <<180/M_PI* X(4,0) << " " << 180/M_PI*X(5,0) <<
                      " " << 180/M_PI*X(6,0) << " " << X(dim.n, X.n_cols-1)  << " " << 180/M_PI*u1(0,1) << " " << u1(1,1) << " " << input <<
                       "\n";

        for(uint iter = 0; iter <  1 + sac_params.T_pre/sac_params.t_sample; iter++)
        {
        sac_result << time_grid[iter] << " ";
        }
        sac_result << "\n";
        sac_result << X_star.row(0) << "\n" << X_star.row(1) << "\n" << 180/M_PI*X_star.row(2) << "\n" << X_star.row(3) << "\n" <<180/M_PI* X_star.row(4) << "\n" << 180/M_PI*X_star.row(5) <<
                      "\n" << 180/M_PI*X_star.row(6) << "\n" << X_star.row(dim.n)  << "\n" << 180/M_PI*u_star.row(0) << "\n" << u_star.row(1) <<"\n";
        // SAVING TORQUE DATA
        // save current control to data.m in matlab notation

        iterator++;

        // new initial state
        for (int iter=0; iter<dim.n; iter++)
        {
            x[iter] = X(iter,1);
        }
        x[dim.n] = 0;


        // update state for next iteration; in matlab code: X = X(2:end,:)
        // this step is only important if sac_params.Tm > 2*sac_params.t_sample
        X.cols(arma::span(0,X.n_cols-2)) = X.cols(arma::span(1,X.n_cols-1));

        // update torque for next time grid, i.e. delete first column and add a zero column at the end
        u1.cols(arma::span(0,u1.n_cols-2)) = u1.cols(arma::span(1,u1.n_cols-1));
        u1.col(u1.n_cols-1).zeros();

        // define new time grid
        for(uint iter = 0; iter < time_grid.size(); iter++)
        {
            time_grid[iter] += sac_params.t_sample;
        }

        // define backward time grid
        time_grid_backwards.clear();
        time_grid_backwards.insert(time_grid_backwards.end(), time_grid.rbegin(), time_grid.rend());
    }

////###### END TC-SAC ##############################################################################################################

    // measure overall execution time
    duration = (std::clock() - all_start) / (double) CLOCKS_PER_SEC;
    std::cout<<"Duration of simulation: "<< duration <<'\n';


    input_file.close();
    sac_result.close();


    // average computation time
    for(uint iter=1; iter<run_time.size(); iter++)
    {
        run_time[0] += run_time[iter];
    }
    run_time[0] = run_time[0]/run_time.size();
    std::cout << "Average computation time of TC-SAC: " << run_time[0] << std::endl;
////####### END OF WRITING DATA ################################################################################################
    return 0;
}
