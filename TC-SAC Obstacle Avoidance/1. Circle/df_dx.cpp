#include "df_dx.h"

using namespace arma;
using namespace std;

void df_dx(state_type& x, state_type& u, mat& Jf)
{

    const double m=1200, theta=2400, l_f=1.35, l_r=1.45,
            mF0 = 621.43, mR0 = 578.57, r=0.3, cw=0.31, rho= 1.293,
            A=2.0, muyf = 0.8, muyr = 1.1, g = 9.81, k=4;

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

    double  u0=u[0],
            u1=u[1];


    double sina=sin(x2+x4);
    double cosa=cos(x2+x4);

    double sinb=sin(x4);
    double cosb=cos(x4);

    double sinc=sin(x4 - x6);
    double cosc=cos(x4 - x6);

    double sind=sin(2*atan((g*mF0)/(2000*C_4)));

    double aa=(1007958012753983*x6);
    double bb=atan2(l_f*x5 + x3*sinb, x3*cosb);
    double cc=(1007958012753983*bb);
    double dd=(aa/17592186044416 - cc/17592186044416);
    double ee=(C_1*g*mF0*muyf);
    double ff=(atan((2*C_3*sind*dd)/ee) - (2*C_3*sind*dd)/ee);
    double gg= atan(C_5*ff + (2*C_3*sind*dd)/ee);
    double hh=cos(C_1*gg);
    double ii=(sinb/(x3*cosb) - (l_f*x5 + x3*sinb)/(pow(x3,2)*cosb));
    double jj=(1007958012753983*C_3*pow(x3,2)*sind*pow(cosb,2)*ii);
    double kk=(8796093022208*C_1*g*mF0*muyf*(pow(x3,2)*pow(cosb,2) + pow((l_f*x5 + x3*sinb),2)));
    double ll=((sinb*(l_f*x5 + x3*sinb))/(x3*pow(cosb,2)) + 1);
    double pp=(1007958012753983*C_3*sin(2*atan((g*mR0)/(2000*C_4)))*atan2(x3*sinb - l_r*x5, x3*cosb));
    double mm=atan(pp/(8796093022208*C_1*g*mR0*muyr));
    double nn=((4*pow(C_3,2)*pow(sind,2)*pow(dd,2))/(pow(C_1,2)*pow(g,2)*pow(mF0,2)*pow(muyf,2)) + 1);
    double oo=(8796093022208*C_1*g*mF0*muyf*nn*(pow(x3,2)*pow(cosb,2) + pow((l_f*x5 + x3*sinb),2)));
    double qq=atan(C_5*(mm - pp/(8796093022208*C_1*g*mR0*muyr)) + pp/(8796093022208*C_1*g*mR0*muyr));
    double rr=(1007958012753983*C_3*pow(x3,2)*sin(2*atan((g*mR0)/(2000*C_4)))*pow(cosb,2)*(sinb/(x3*cosb) + (l_r*x5 - x3*sinb)/(pow(x3,2)*cosb)));
    double ss=(8796093022208*C_1*g*mR0*muyr*(pow(x3,2)*pow(cosb,2) + pow((l_r*x5 - x3*sinb),2)));
    double tt=(1015979355474958556092082364289*pow(C_3,2)*pow(sin(2*atan((g*mR0)/(2000*C_4))),2)*pow(atan2(x3*sinb - l_r*x5, x3*cosb),2) );
    double uu=(tt/(77371252455336267181195264*pow(C_1,2)*pow(g,2)*pow(mR0,2)*pow(muyr,2)) + 1);
    double vv=(8796093022208*C_1*g*mR0*muyr*(pow(x3,2)*pow(cosb,2) + pow((l_r*x5 - x3*sinb),2))*uu);
    double ww=pow((C_5*(mm - pp/(8796093022208*C_1*g*mR0*muyr)) + pp/(8796093022208*C_1*g*mR0*muyr)),2);
    double xx=(1007958012753983*C_3*pow(x3,2)*sin(2*atan((g*mR0)/(2000*C_4)))*pow(cosb,2)*((sinb*(l_r*x5 - x3*sinb))/(x3*pow(cosb,2)) - 1));
    double yy=(1007958012753983*C_3*l_r*x3*sin(2*atan((g*mR0)/(2000*C_4)))*cosb);
    double zz=((1007958012753983*C_3*sind)/(8796093022208*C_1*g*mF0*muyf) - (1007958012753983*C_3*sind)/(8796093022208*C_1*g*mF0*muyf*nn));
    double aaa=(C_5*zz - (1007958012753983*C_3*sind)/(8796093022208*C_1*g*mF0*muyf));
    double bbb=((1007958012753983*C_3*l_f*x3*sind*cosb)/kk - (1007958012753983*C_3*l_f*x3*sind*cosb)/oo);
    double ccc=(C_5*bbb - (1007958012753983*C_3*l_f*x3*sind*cosb)/kk);
    double ddd=((1007958012753983*C_3*pow(x3,2)*sind*pow(cosb,2)*ll)/kk - (1007958012753983*C_3*pow(x3,2)*sind*pow(cosb,2)*ll)/oo);
    double eee=(C_5*ddd - (1007958012753983*C_3*pow(x3,2)*sind*pow(cosb,2)*ll)/kk);
    double fff=(pow((C_5*ff + (2*C_3*sind*dd)/ee),2) + 1);
    Jf(0,2)=-x3*sina;
    Jf(0,3)=cosa;
    Jf(0,4)=-x3*sina;

    Jf(1,2)=x3*cosa;
    Jf(1,3)=sina;
    Jf(1,4)=x3*cosa;

    Jf(2,5)=1;


        Jf(3,3) = (- A*cw*rho*x3 + (C_1*g*mF0*muyf*sinc*hh*(C_5*(jj/kk - jj/oo) - jj/kk))/fff + (C_1*g*mR0*muyr*cos(C_1*qq)*sinb*(C_5*(rr/ss - rr/vv) - rr/ss))/(ww + 1))/m;
        Jf(3,4) = -((u1*sinb)/r + (u1*sinc)/r - g*mF0*muyf*cosc*sin(C_1*gg) + g*mR0*muyr*sin(C_1*qq)*cosb + (C_1*g*mR0*muyr*cos(C_1*qq)*sinb*(C_5*(xx/ss - xx/vv) - xx/ss))/(ww + 1) - (C_1*g*mF0*muyf*sinc*hh*eee)/fff)/m;
        Jf(3,5) = -((C_1*g*mR0*muyr*cos(C_1*qq)*sinb*(C_5*(yy/ss - yy/vv) - yy/ss))/(ww + 1) - (C_1*g*mF0*muyf*sinc*hh*ccc)/fff)/m;
        Jf(3,6) = -(- (u1*sinc)/r + g*mF0*muyf*cosc*sin(C_1*gg) + (C_1*g*mF0*muyf*sinc*hh*aaa)/fff)/m;

        Jf(4,3) = ((u1*sinb)/r + m*x3*x5 + (u1*sinc)/r - g*mF0*muyf*cosc*sin(C_1*gg) + g*mR0*muyr*sin(C_1*qq)*cosb)/(m*pow(x3,2)) + (- m*x5 + (C_1*g*mF0*muyf*cosc*hh*(C_5*(jj/kk - jj/oo) - jj/kk))/fff + (C_1*g*mR0*muyr*cos(C_1*qq)*cosb*(C_5*(rr/ss - rr/vv) - rr/ss))/(ww + 1))/(m*x3);
        Jf(4,4) = -((u1*cosb)/r + (u1*cosc)/r + g*mF0*muyf*sinc*sin(C_1*gg) - g*mR0*muyr*sin(C_1*qq)*sinb + (C_1*g*mR0*muyr*cos(C_1*qq)*cosb*(C_5*(xx/ss - xx/vv) - xx/ss))/(ww + 1) - (C_1*g*mF0*muyf*cosc*hh*eee)/fff)/(m*x3);
        Jf(4,5) = -(m*x3 + (C_1*g*mR0*muyr*cos(C_1*qq)*cosb*(C_5*(yy/ss - yy/vv) - yy/ss))/(ww + 1) - (C_1*g*mF0*muyf*cosc*hh*ccc)/fff)/(m*x3);
        Jf(4,6) = ((u1*cosc)/r + g*mF0*muyf*sinc*sin(C_1*gg) - (C_1*g*mF0*muyf*cosc*hh*aaa)/fff)/(m*x3);

        Jf(5,3) = -((C_1*g*l_r*mR0*muyr*cos(C_1*qq)*(C_5*(rr/ss - rr/vv) - rr/ss))/(ww + 1) - (C_1*g*l_f*mF0*muyf*hh*cos(x6)*(C_5*(jj/kk - jj/oo) - jj/kk))/fff)/theta;
        Jf(5,4) = ((C_1*g*l_r*mR0*muyr*cos(C_1*qq)*(C_5*(xx/ss - xx/vv) - xx/ss))/(ww + 1) + (C_1*g*l_f*mF0*muyf*hh*cos(x6)*eee)/fff)/theta;
        Jf(5,5) = ((C_1*g*l_r*mR0*muyr*cos(C_1*qq)*(C_5*(yy/ss - yy/vv) - yy/ss))/(ww + 1) + (C_1*g*l_f*mF0*muyf*hh*cos(x6)*ccc)/fff)/theta;
        Jf(5,6) = -(- (l_f*u1*cos(x6))/r + g*l_f*mF0*muyf*sin(C_1*gg)*sin(x6) + (C_1*g*l_f*mF0*muyf*hh*cos(x6)*aaa)/fff)/theta;

    Jf(6,6)=-k;

}
