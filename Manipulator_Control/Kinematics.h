#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <iostream>
#include <math.h>
#include <Characteristics.h>
#include <opencv/cv.h>

# define M_PI           3.14159265358979323846  /* pi */

namespace kin {
struct state {
    double x, y, z;
    double vx, vy, vz;
};

class ForwardKinematics
{
public:
   ForwardKinematics();
   kin::state get(double angle);
private:
   struct dhParams {
       dhParams(int l, double t, double _d, double _a, double al)
       {
           link = l;
           theta = t;
           d = _d;
           a = _a;
           alpha = al;
       }

       int link;
       double theta;
       double d;
       double a;
       double alpha;
   };
   struct dhVals{
       double q1, q2, q3, q4, q5, q6, q7, q8, q9;
       double L1, L2, L3, L4, L5, L6, L7, L8, L9;
   }dhvals;

   std::vector<dhParams> dhMatrix;//array of dh parameters - dhMatrix is the variable I use

   cv::Mat dh;
   cv::Mat getAMatrix(dhParams dh);
   void initDh(double angle);
};

class InverseKinematics
{
public:
   InverseKinematics();
   void get(kin::state& legState);
   void updateCurrentState(double x, double y, double z);

private:
   kin::state currentState = {0,0,0,0,0,0};
};

class Kinematics
{
public:
    Kinematics();
    state process();
    ForwardKinematics fkin;
    InverseKinematics ikin;
};
};

#endif // KINEMATICS_H
