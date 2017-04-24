#include "Characteristics.h"

Characteristics::Characteristics()
{
}

long Characteristics::stepsToMicron(int steps, double actuatorPitch, double motorDeg, int microStepping){
    double stepsPerRev = microStepping*(360/motorDeg);
    double totalRev = steps/stepsPerRev;
    return totalRev*actuatorPitch*1000;

}

double Characteristics::stepsToMillimeters(int steps, double actuatorPitch, double motorDeg, int microStepping){
    double stepsPerRev = microStepping*(360/motorDeg);
    double totalRev = steps/stepsPerRev;
    return totalRev*actuatorPitch;
}

long Characteristics::millimetersToSteps(double mm, double actuatorPitch, double motorDeg, int microStepping){
    return (microStepping/actuatorPitch)*mm;


}

long Characteristics::micronToSteps(double micron, double actuatorPitch, double motorDeg, int microStepping){
    return (microStepping/actuatorPitch)*micron/1000;
}
