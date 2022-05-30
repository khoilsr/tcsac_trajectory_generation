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
         x_d[0]=50*sin(2*M_PI*1/3*0.075*time);
         x_d[1]=50*cos(2*M_PI*1/2*0.075*time);


            x_d[2]=0;
            x_d[3]=sqrt(pow(50*sin(2*M_PI*1/3*0.075*(time+0.001))-x_d[0],2)+pow(50*cos(2*M_PI*1/2*0.075*(time+0.001))-x_d[1],2))/0.001;
            x_d[5]=0;
            x_d[6]=0;

    return x_d;
}
