#ifndef Drive_H
#define Drive_H
#include "Drive.h"
#include "rev/CANSparkMax.h"

#pragma once


using namespace std;

class Drive {
    private:
        // Define motor, sensors, and pnematic pointers here

        rev::CANSparkMax * m_leftdrive;
        rev::CANSparkMax * m_leftdrive2;
        rev::CANSparkMax * m_rightdrive;
        rev::CANSparkMax * m_rightdrive2;

    public:
        Drive();
        void Joystick_Drive(double LeftStick, double RightStick);
};

#endif