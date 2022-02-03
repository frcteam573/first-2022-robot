#ifndef Log_H
#define Log_H

#pragma once

#include "Robot.h"
#include "Log.h"
#include "frc/PowerDistribution.h"

using namespace std;

class Log {

    private:
    // Define motor, sensors, and pnematic pointers here
    frc::PowerDistribution * Board;

    public:
    Log();  
     // Define Log class functions here 
    void Dashboard();
    void PDPTotal();
    void CurrentCompare(int slot,double PWMin);
};

#endif