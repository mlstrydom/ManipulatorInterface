#include "Motors.h"

Motors::Motors()
{
}

std::string Motors::commandFactory(std::string motor, long steps)
{
    std::ostringstream move;
    move << "$ " << motor << " " << steps << " #"  ;//String for motor command
    return move.str();
}
