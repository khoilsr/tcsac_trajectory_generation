#include "reference_trajectory.h"
#include <iostream>



double correct_time( double time, double t_step)
{
    // correct time greater than right bound
    if ( time > t_step )
    {
        while( (time > t_step) )
        {
            time = time - t_step;
        }
    }



    return time;
}

std::vector< double > reference_trajectory( double time )
{
    std::vector< double > x_d (7);
    double v=10;
    double t_step=100;
    time=correct_time(time,t_step);
    // Square Path
    if(time < t_step/4)
    {
        x_d[0] = v*time;
        x_d[1] = 0;
        x_d[2]=0;

    }
    else if(time < t_step/2)
    {
        x_d[0] = v*t_step/4;
        x_d[1] = v*(time-t_step/4);
        x_d[2]=90*M_PI/180;
    }
    else if(time < 3*t_step/4)
    {
        x_d[0] = v*t_step/4-(v*(time-t_step/2));
        x_d[1] = v*t_step/4;
        x_d[2]=180*M_PI/180;
    }
    else if(time <=t_step)
    {
        x_d[0] = 0;
        x_d[1] = v*t_step/4-(v*(time-3*t_step/4));
        x_d[2]=270*M_PI/180;
    }

            x_d[2]=0;
            x_d[3]=v;
            //std::cout<<x_d[3]<<std::endl;
            x_d[5]=0;
            x_d[6]=0;

    return x_d;
}
