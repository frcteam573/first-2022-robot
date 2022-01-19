#ifndef Drive_H
#define Drive_H
#include "Drive.h"
#include "rev/CANSparkMax.h"
#include "frc/DoubleSolenoid.h"

#pragma once


using namespace std;

class Drive {
    private:
        // Define motor, sensors, and pnematic pointers here

        rev::CANSparkMax * m_leftdrive;
        rev::CANSparkMax * m_leftdrive2;
        rev::CANSparkMax * m_rightdrive;
        rev::CANSparkMax * m_rightdrive2;
        rev::CANSparkMax * m_leftclimb;
        rev::CANSparkMax * m_rightclimb;
        frc::DoubleSolenoid * p_climberlock;
        frc::DoubleSolenoid * p_climbertilt;

    
    public:
        Drive();
        void Joystick_Drive(double LeftStick, double RightStick);
        void climber_extend();
        void climber_retract();
        void climber_hold();
        void climber_tiltin();
        void climber_tiltout(); 
};

#endif