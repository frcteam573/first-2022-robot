#ifndef Led_H
#define Led_H

#pragma once

#include "Led.h"
#include "frc/motorcontrol/Spark.h"


using namespace std;

class Led {

    private:
    // Define motor, sensors, and pnematic pointers here
    frc::Spark * m_leds;


    public:
    Led();  
     // Define Led class functions here 
     void led_control(std::string input);

};

#endif