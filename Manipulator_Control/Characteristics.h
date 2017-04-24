#ifndef CHARACTERISTICS_H
#define CHARACTERISTICS_H

class Characteristics
{
public:
    Characteristics();
    struct dhParameters {
        double param1 = 1.0;
        double param2 = 0.0;
        double param3 = 0.01;
    };

    struct actuator {
        double screwPitch;
        double motorDegPerStep;
        double motorMicroStep;
        double screwLength ;
    };

    actuator s, l;
    long stepsToMicron(int steps, double actuatorPitch, double motorDeg, int microStepping);
    double stepsToMillimeters(int steps, double actuatorPitch, double motorDeg, int microStepping);
    long millimetersToSteps(double mm, double actuatorPitch, double motorDeg, int microStepping);
    long micronToSteps(double micron, double actuatorPitch, double motorDeg, int microStepping);
};

#endif // CHARACTERISTICS_H
