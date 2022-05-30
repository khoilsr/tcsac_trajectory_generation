############################################################
##### INSTRTUCTIONS FOR USE OF TC-SAC ######################
############################################################

Before applying TC-SAC to your system, the following steps have to be taken first:

STEP 1: Define Core Paramaters
	Step 1.1: Dimensions of problem
	In 'main.cpp', define the numder of states 'dim.n' and the number of inputs 'dim.m'.


STEP 2: Forward Integration ('simu_state.cpp')
	Step 2.2: Modify ODE describing system
	Change the ODE describing your system in 'simu_state.cpp'. The ODE is defined in the public part of 'class STATE_EQ'. 
	Step 2.3: Change cost function
	The last entry of the ODE given in 'class STATE_EQ' is the cost function. By default, the cost function consists of a tracking, joint limitation and obstacle avoidance 	part (for how to define the paramaters see STEP 9). You need to provide the calculation for the distance between each grid point and the obstacle in the public section 	of 'class STATE_EQ'.
	Step 2.4: Adjust angle correction
	In the 'simu_state' function correct all angles at the corresponding part in the code (marked by ////#####)
	Step 2.5: Additional optional steps
	- In case your ODE consists of additional parameters that change over time, add their name and type in the header file 'definitions.h' to the struct 'ODE_PARAMS'. You 		can either set them in 'main.cpp' (see STEP 9) or in the actual 'simu_state' function. We recommend the first.
	- In 'void simu_state(...)' you may change the used stepper. The stepper used by default is 'runge_kutte4'.

STEP 3: Define Jacobian Matrix Df_dx ('df_dx.cpp')
	In 'df_dx.cpp', define the jacobian matrix Jf (derivative w.r.t. state). You only need to provide the non-zero elements. The first row of Jf consists of the gradient of 		the first entry in your ODE for the state.

STEP 4: Define Jacobian Matrix Df_du ('df_du.cpp')
	In 'df_du.cpp', define the jacobian matrix Jf (derivative w.r.t. input). Only non-zero elements need to be defined. Each row of Jf is the gradient of the corresponding 	entry in the ODE of the system.

STEP 5: Define Jacobian Matrix Dl1_dx ('dl1_dx.cpp')
	In 'dl1_dx.cpp', define the jacobian matrix Jl1 (derivative w.r.t. state). Only non-zero elements need to be defined.
	
STEP 6: Define Jacobian Matrix Dl1_du ('dl1_du.cpp')
	In 'dl1_du.cpp', define the jacobian matrix Jl1 (derivative w.r.t. input). Only non-zero elements need to be defined.

STEP 9: Define Parameters
	In 'main.cpp', define all the parameters in the corresponding section. The parameters are saved in structures indicating to what part of the code they are belonging.
########SAC Parameters:	
----------------sac_params.t_sample	: Sampling time
					  Should be a rational number
					  Data type: double
----------------sac_params.Tm_sample	: Application time of optimal control computed by SAC
					  Data type: double
----------------sac_params.T_pre	: Prediction horizon
					  Data type: double
----------------sac_params.k_max	: Maximum number of line search iterations for determining the application time of the optimal control computed by SAC
					  Data type: double
----------------sac_params.omega	: Regulates application time of optimal control computed by SAC in the line search
					  Data type: double
----------------sac_params.w_alpha	: Factor for specifying desired sensitivity
					  Data type: double
----------------sac_params.S		: Matrix for calculating optimal control by SAC
					  Dimensions are (dim.m x dim.m)
					  Data type: arma::mat
########Bryson/Ho Parameters	
----------------bh_params.W		: Weighting matrix for calculating control update based on Bryson/Ho
					  Dimension is (dim.m x dim.m)
					  Data type: arma::mat
----------------bh_params.tol_psi	: Scaling factor when calculating differene between actual final position and desired one
					  Should be smaller than 1
					  Data type: double
########Cost Parameters
----------------cost_params.P1		: Weighting Matrix for final cost
					  Dimension is (dim.n x dim.n)
					  So far only a diagonal matrix is supported
					  Data type: std::vector<double>		  
----------------cost_params.Q1		: Weighting Matrix for tracking cost
					  Dimension is (dim.n x dim.n)
					  So far only a diagonal matrix is supported
					  Data type: std::vector<double>
----------------cost_params.c_max	: Shape parameter for joint limit cost in order to not violate maximal possible angle
					  Data type: double
----------------cost_params.c_min	: Shape parameter for joint limit cost in order to not violate minimal possible angle
					  Data type: double
----------------cost_params.theta_max	: Maximal possible angle for joints of robot
					  If there is no limitation to a link set the corresponding value to 360*M_PI/180
					  Data type: std::vector<double>
----------------cost_params.theta_min	: Minimal possible angle for joints of robot
					  If there is no limitation to a link set the corresponding value to -360*M_PI/180
					  Data type: std::vector<double>
----------------cost_params.theta_tol	: Tolerance for determining when to start punishing joint limit violation
					  Needs to be defined for each link
					  Data type: std::vector<double>
----------------cost_params.V_max	: Shape parameter of cost for obstacle avoidance
					  Data type: double
----------------cost_params.sigma	: Shape parameter of cost for obstacle avoidance
					  Data type: double
----------------cost_params.obstacle	: Cartesian coordinates of center of sperical obstacle
					  Data type: std::vector<double>
########Torque Limits
----------------tor_limit.min		: Minimal Torque; Currently only one entry for all inputs supported
					  Data type: std::vector<double>
----------------tor_limit.max		: Maximal Torque; Currently only one entry for all inputs supported
					  Data type: std::vector<double>
########Initial Position
----------------x			: Initial position of robot
					  Only non-zero entries need to be defined
					  Data type: std::vector<double>
########Desired Position
----------------x_d			: Desired position of the robot
					  Only non-zero entries need to be defined
					  Data type: std::vector<double>
########Simulation Time
----------------t_stop			: Simulation time

##############################################################################################################################################
General information on the used libraries odeint and armadillo can be found here:
odeint:		http://headmyshoulder.github.io/odeint-v2/
		http://headmyshoulder.github.io/odeint-v2/doc/index.html
armadillo:	http://arma.sourceforge.net/docs.html	
