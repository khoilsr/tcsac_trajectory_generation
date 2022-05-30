#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <vector>

#include <armadillo>
#include "spline.h"
#include <complex>
typedef std::vector< double > state_type;
extern state_type x;

// --------------- STRUCTURES ----------------------------------------------------------------------
// problem dimensions
struct DIM {
    int n;
    int m;
};

// parameters for sac
struct SAC_PARAMS
{
    double t_sample;
    double rnd_factor;
    double Tm_sample;
    double T_pre;
    double delta_t;
    double k_max;
    double omega;
    double w_alpha;
    arma::mat S;
};

// parameters of cost
struct COST_PARAMS
{
    std::vector<double> P1;
    std::vector<double> Q1;
    std::vector<double> Q1_control;

    double c_max;
    double c_min;
    state_type theta_max;
    state_type theta_min;
    state_type theta_tol;

    state_type obstacle_avoid;
    state_type obstacle_x;
    state_type obstacle_y;
    state_type obstacle_r;

    bool trajectory_tracking;

};

// parameters related to bryson/ho
struct BH_PARAMS
{
    arma::mat W;
    double tol_psi;
};

// max/min torque
struct INPUT_LIMIT
{
    std::vector< double > max;
    std::vector< double > min;
};

// --------------------------------------------------------------------------------------------------------------------------------------------
// parameters for odes
struct ODE_PARAMS
{
    int n;
    int m;

    double t_sample;

    std::vector< double > time;
    std::vector< double > x_d;
    std::vector< double > Q;
    std::vector< double > Q_control;


    double c_max;
    double c_min;
    state_type theta_max;
    state_type theta_min;
    state_type theta_tol;

    state_type obstacle_avoid;
    state_type obstacle_x;
    state_type obstacle_y;
    state_type obstacle_r;

    arma::mat W;

    struct COST_PARAMS cost_params;
    struct INPUT_LIMIT input_limit;
    bool trajectory_tracking;
    std::vector< tk::spline> s;
};

// --------------------------------------------------------------------------------------------------------------------------------------------

extern DIM dim;
extern SAC_PARAMS sac_params;
extern COST_PARAMS cost_params;
extern BH_PARAMS bh_params;
extern INPUT_LIMIT input_limit;
extern ODE_PARAMS ode_params;

#endif // DEFINITIONS_H
