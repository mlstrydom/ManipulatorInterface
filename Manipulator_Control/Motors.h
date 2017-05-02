#ifndef MOTORS_H
#define MOTORS_H
#include <string>
#include <iostream>     // std::cout, std::ios
#include <sstream>      // std::ostringstream

class Motors
{
public:
    Motors();
    std::string commandFactory(std::string motor, long steps, long velocity);
};

#endif // MOTORS_H
