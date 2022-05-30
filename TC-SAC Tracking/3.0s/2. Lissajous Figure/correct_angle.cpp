#include "correct_angle.h"
#include <iostream>

double correct_angle( double angle )
{
    // correct angle greater than right bound

        while( (angle > 2*M_PI) )
        {
            angle = angle - 2*M_PI;
        }
    // correct angle smaller than left bound

        while ( angle < -2*M_PI )
        {
            angle = angle + 2*M_PI;
        }
    return angle;
}
