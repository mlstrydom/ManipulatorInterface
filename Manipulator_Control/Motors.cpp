#include "Motors.h"

Motors::Motors()
{
}

std::string Motors::commandFactory(std::string motor, long steps, long velocity)
{
    std::ostringstream move;
    move << "$ " << motor << " " << steps  << " " << velocity << " #"  ;//String for motor command
//    std::cout << "string send to UNO" << move.str() << std::endl;
    return move.str();
}
