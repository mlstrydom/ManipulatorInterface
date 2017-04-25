#include "Kinematics.h"
using namespace kin;

Kinematics::Kinematics()
{
}

kin::state Kinematics::process()
{
    kin::state s = fkin.get(M_PI/2);
    ikin.get(s);
    return s;
}

kin::ForwardKinematics::ForwardKinematics()
{

}

cv::Mat kin::ForwardKinematics::getAMatrix(dhParams dh){
    double a = dh.a;
    double alpha = dh.alpha;
    double d = dh.d;
    double theta = dh.theta;

    float m[4][4] = {
        {cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)},
        {sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)},
        {0, sin(alpha), cos(alpha), d},
        {0, 0, 0, 1}
    };

    cv::Mat mat_m = cv::Mat(4, 4, CV_32FC1, &m).clone();
    return mat_m;
}



void ForwardKinematics::initDh(double angle)
{
    dhvals.q1 = 0;
    dhvals.q2 = 0;
    dhvals.q3 = 0;
    dhvals.q4 = angle;
    dhvals.q5 = 0;
    dhvals.q6 = 0;
    dhvals.q7 = 0;
    dhvals.q8 = 0;
    dhvals.q9 = 0;
    dhvals.L1 = 0;
    dhvals.L2 = 0;
    dhvals.L3 = 400;
    dhvals.L4 = 0;
    dhvals.L5 = 0;
    dhvals.L6 = 0;
    dhvals.L7 = 0;
    dhvals.L8 = 0;
    dhvals.L9 = 500;

    dhMatrix.clear();
    dhMatrix.push_back(dhParams(1, dhvals.q1, dhvals.L1, 0, -M_PI/2));
    dhMatrix.push_back(dhParams(2, dhvals.q2, 0, dhvals.L2, -M_PI/2));
    dhMatrix.push_back(dhParams(3, dhvals.q3, 0, dhvals.L3, -M_PI/2));
    dhMatrix.push_back(dhParams(4, dhvals.q4, 0, dhvals.L4, 0));
    dhMatrix.push_back(dhParams(5, 0, 0, dhvals.q5, M_PI/2));
    dhMatrix.push_back(dhParams(6, 0, 0, dhvals.q6, 0));
    dhMatrix.push_back(dhParams(7, dhvals.q7, 0, 0, M_PI/2));
    dhMatrix.push_back(dhParams(8, 0, 0, 0, 0));
    dhMatrix.push_back(dhParams(9, 0, dhvals.L9, 0, -M_PI/2));
}

kin::state kin::ForwardKinematics::get(double angle)
{
    initDh(angle);

    cv::Mat tf;

    bool first = true;
    for(auto d : dhMatrix){//auto d : itterate through every element of dhMatrix array
        if(first){
            tf = getAMatrix(d);//pass d to dh parameter in getAMatrix
            first = false;
        }else{
            tf = tf * getAMatrix(d);
        }
    }
    std::cout << "Transfer Functoin tf from matrix = "<< std::endl << " "  << tf << std::endl << std::endl;
    std::cout << "s.x from matrix = " << tf.at<float>(0,3) << std::endl;
    std::cout << "s.y from matrix = " << tf.at<float>(1,3) << std::endl;
    std::cout << "s.z from matrix = " << tf.at<float>(2,3) << std::endl;

    kin::state s;
    s.x = tf.at<float>(0,3);
    s.y = tf.at<float>(1,3);
    s.z = tf.at<float>(2,3);

    return s;
}

kin::InverseKinematics::InverseKinematics()
{

}

void kin::InverseKinematics::get(kin::state &legState)
{
    double velocity = 1;
    //TODO: check sign of y direction
    double beta = atan((legState.y-currentState.y)/(legState.z-currentState.z));
    //Use the forwardKinematics to generate velocities...
    legState.vx = 0;
    legState.vy = velocity*cos(beta);
    legState.vz = velocity*sin(beta);

    //TODO: read the real position from the arduino!
    currentState = legState;
}

void InverseKinematics::updateCurrentState(double x, double y, double z)
{
    currentState.x = x;
    currentState.y = y;
    currentState.z = z;
}
