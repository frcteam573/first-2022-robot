#ifndef Appendage_H
#define Appendage_H

#pragma once
#include "rev/CANSparkMax.h"
#include "rev/SparkMaxRelativeEncoder.h"
#include <frc/DoubleSolenoid.h>
#include "frc/smartdashboard/SmartDashboard.h"

using namespace std;

class Appendage
{
private:
    // Define motor, sensors, and pnematic pointers here
    frc::DoubleSolenoid *p_Intake;

    rev::CANSparkMax *m_Intake1;
    rev::CANSparkMax *m_Intake2;
    rev::CANSparkMax *m_Shooter1;
    rev::CANSparkMax *m_Shooter2;
    rev::CANSparkMax *m_Feeder;
    rev::CANSparkMax *m_Susan;
    rev::CANSparkMax *m_Hood;

    rev::SparkMaxRelativeEncoder *s_Shooter_Encoder;
    rev::SparkMaxRelativeEncoder *s_Susan_Encoder;
    rev::SparkMaxRelativeEncoder *s_Hood_Encoder;
   

public:
    Appendage();
    void Intake_In();
    void Intake_Out();
    void Intake_Off();

    void Feeder_In();
    void Feeder_Out();
    void Feeder_Off();

    void Intake_Up();
    void Intake_Down();
    double Remap_Val(double i, double threshold);

    bool Shooter_Encoder();
    void Shooter_Off();

    double Get_Distance(double camera_y);
    bool Rotate(double camera_exists, double camera_x, bool direction);
    void Rotate_Off();
    double Articulate(double distance);


    void DashboardCreate();

    static double shooter_p_in;
    static double shooter_target_in;
    static double shooter_f_in;

    void dashboard();



};
#endif