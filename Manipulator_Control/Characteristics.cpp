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
    double stepsPerRev = microStepping*(360/motorDeg);//(360/1.8)*4=800 - steps per rev
    double totalRev = steps/stepsPerRev; //for 8000 steps : 16000/800 = 20 rev
    return totalRev*actuatorPitch; //travel distance = 20mm/rev*rev = 20*20=400mm
}

long Characteristics::millimetersToSteps(double mm, double actuatorPitch, double motorDeg, int microStepping){
    double stepsPerRev = microStepping*(360/motorDeg);
    return (stepsPerRev/actuatorPitch)*mm;


}

long Characteristics::micronToSteps(double micron, double actuatorPitch, double motorDeg, int microStepping){
    double stepsPerRev = microStepping*(360/motorDeg);
    return (stepsPerRev/actuatorPitch)*micron/1000;
}
