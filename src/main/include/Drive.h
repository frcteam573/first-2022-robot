#ifndef Drive_H
#define Drive_H

#include "Drive.h"
#include "rev/CANSparkMax.h"
#include "frc/DoubleSolenoid.h"
#include "frc/ADXRS450_Gyro.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "rev/SparkMaxRelativeEncoder.h"

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
        frc::ADXRS450_Gyro * s_gyro;
        rev::SparkMaxRelativeEncoder * s_leftclimber_enc;
        rev::SparkMaxRelativeEncoder * s_rightclimber_enc;
        rev::SparkMaxRelativeEncoder * s_rightdrive_enc;
        rev::SparkMaxRelativeEncoder * s_leftdrive_enc;   

    public:
        Drive();
        void Joystick_Drive(double LeftStick, double RightStick);
        int climber_extend();
        int climber_retract();
        void climber_hold();
        void climber_tiltin();
        void climber_tiltout();
        void camera_intake(double camera_x, double joystick_y);
        void drive_straight(bool first, double joystick_y);
        void dashboard();
        void gyro_reset();
        double deadband(double input, double deadband_size);
        void drive_PID(vector<double>value_in, int count);
        tuple <bool, bool> climber_setpoint(string input);      
        double Remap_Val(double i, double threshold);
        void DashboardCreate();
        static double kvelo_in;
        static double kpos_in;
        static double kph_in;

        int climb_lock;
       
};

#endif