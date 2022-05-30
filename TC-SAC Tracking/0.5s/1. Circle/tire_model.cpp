//#include "tire_model.h"
//#include <iostream>


//double tire_model(double alpha,double Fz,double muy)
//{
//double C_1 = 0.5;
//double C_2 = 600;
//double C_3 = 3000;
//double C_4 = 40;
//double C_5 = -1;

//    double ALPHA = 180 / M_PI * alpha;

//    Fz = Fz/1000;
//    muy = muy * 1000;
//    double D = C_2 * Fz;
//    double BCD = C_3*sin(2*atan(Fz/C_4));
//    double B = BCD/(C_1*D);
//    double ALPHAeq = C_2/muy*ALPHA;
//    double fy = D * sin(C_1 * atan(B * ALPHAeq - C_5*(B * ALPHAeq - atan(B * ALPHAeq))));
//    double Fy = -muy/C_2*fy;


//return Fy;
//}



#include "tire_model.h"
#include <iostream>


double tire_model(double alpha,double Fz,double muy)
{
double C_1 = 0.463;
double C_2 = 600;
double C_3 = 3000;
double C_4 = 40;
double C_5 = -1;


double alph=correct_alpha(alpha);
double ALPHA,D,BCD,B,ALPHAeq,fy,Fy;



    ALPHA = 180 / M_PI * alph;

    Fz = Fz/1000;
    muy = muy * 1000;
    D = C_2 * Fz;
    BCD = C_3*sin(2*atan(Fz/C_4));
    B = BCD/(C_1*D);
    ALPHAeq = C_2/muy*ALPHA;
    fy = D * sin(C_1 * atan(B * ALPHAeq - C_5*(B * ALPHAeq - atan(B * ALPHAeq))));
    Fy = -muy/C_2*fy;
return Fy;

}

