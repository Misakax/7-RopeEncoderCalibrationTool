#include "CalibrationV3Analytic.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {
constexpr double pi=3.1415926535897932384626433832795;
}

int main()
{
    const int N=7;
    double a[N]={0,0,0,0,0,0,0};
    double d[N]={0.0000006,0.0505064,0.4911552,-0.1186827,0.3178316,-0.0007692,0.0927026};
    double alpha[N]={pi/2,-pi/2,pi/2,-pi/2,pi/2,-pi/2,0};
    double theta[N]={0,0,0,0,0,0,0};
    double beta[N]={0,0,0,0,0,0,0};
    double q0[N]={0,0,0,0,0,0,0};
    double q[N]={-0.211202,-0.048625,0.088436,-1.480755,-0.309080,-0.069359,-0.087825};
    double fixture[3]={0,0,0.049};

    std::vector<double> J(3*4*N);
    double P[3];
    if(CalibrationV3EvaluatePointAndJacobian(N,a,d,alpha,theta,beta,q0,q,fixture,P,J.data())!=0)
        return 2;

    double maxAbs=0.0, maxRel=0.0;
    int worstCol=-1, worstRow=-1;

    for(int col=0;col<4*N;++col)
    {
        double* target=nullptr;
        int idx=col%N;
        if(col<N) target=&a[idx];
        else if(col<2*N) target=&d[idx];
        else if(col<3*N) target=&alpha[idx];
        else target=&q0[idx];

        const bool angular = col>=2*N;
        const double eps = angular ? 1e-7 : 1e-7; // rad or m; diagnostic only
        const double old=*target;
        *target=old+eps;
        std::vector<double> Jtmp(3*4*N); double Pp[3];
        CalibrationV3EvaluatePointAndJacobian(N,a,d,alpha,theta,beta,q0,q,fixture,Pp,Jtmp.data());
        *target=old-eps;
        double Pm[3];
        CalibrationV3EvaluatePointAndJacobian(N,a,d,alpha,theta,beta,q0,q,fixture,Pm,Jtmp.data());
        *target=old;

        for(int r=0;r<3;++r)
        {
            double num=(Pp[r]-Pm[r])/(2*eps);
            double ana=J[r*(4*N)+col];
            double ae=std::abs(num-ana);
            double re=ae/std::max(1e-12,std::max(std::abs(num),std::abs(ana)));
            if(ae>maxAbs){maxAbs=ae;worstCol=col;worstRow=r;}
            maxRel=std::max(maxRel,re);
        }
    }

    std::cout<<std::setprecision(12);
    std::cout<<"P="<<P[0]<<","<<P[1]<<","<<P[2]<<"\n";
    std::cout<<"maxAbsDerivativeError="<<maxAbs<<" maxRelDerivativeError="<<maxRel
             <<" worstRow="<<worstRow<<" worstCol="<<worstCol<<"\n";
    std::cout<<"q7 column norm=";
    double n=0; int c=3*N+6;
    for(int r=0;r<3;++r)n+=J[r*(4*N)+c]*J[r*(4*N)+c];
    std::cout<<std::sqrt(n)<<"\n";
    return 0;
}
