#include "finalcost.h"

void finalcost(state_type &x, state_type &x_d, COST_PARAMS cost_params)
{
    // calc final cost and add to already existing

    for (int iter = 0; iter < x_d.size(); iter++)
    {
        x[x.size() - 1] += cost_params.P1[iter] * pow(x[iter] - x_d[iter], 2);
    }

//std::cout << x[x.size() - 1] << std::endl;
//    x[x.size() - 1] += cost_params.P1[0]*pow(x[0] - x_d[0],2);
//    //std::cout << cost_params.P1[0]*pow(x[0] - x_d[0],2)<<std::endl;
//    x[x.size() - 1] += cost_params.P1[1]*pow(x[1] - x_d[1],2);
//    //std::cout << cost_params.P1[1]*pow(x[1] - x_d[1],2)<<std::endl;
//    x[x.size() - 1] += cost_params.P1[2]*pow(x[2] - atan((x[1]-x_d[1])/(x[0]-x_d[0])),2);
//    //std::cout << cost_params.P1[2]*pow(x[2] - atan((x[1]-x_d[1])/(x[0]-x_d[0])),2)<<std::endl;

//    x[x.size() - 1] += cost_params.P1[3]*pow(x[3] - x_d[3],2);
//    x[x.size() - 1] += cost_params.P1[4]*pow(x[4] - x_d[4],2);
//    std::cout<<cost_params.P1[4]<<std::endl;
//    std::cout<<x[0]<<std::endl;
//    std::cout<<x_d[0]<<std::endl;
    //std::cout<<x[x.size() -1]<<std::endl;
}

