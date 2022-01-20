#ifndef Appendage_H
#define Appendage_H

#pragma once
#include "rev/CANSparkMax.h"
#include "rev/SparkMaxRelativeEncoder.h"
#include <frc/DoubleSolenoid.h>

using namespace std;

class Appendage
{
private:
    // Define motor, sensors, and pnematic pointers here
    rev::CANSparkMax *m_Intake;
    frc::DoubleSolenoid *p_Intake;
    rev::CANSparkMax *m_Shooter;
    rev::SparkMaxRelativeEncoder *s_Shooter_Encoder;

public:
    Appendage();
    void Intake_In();
    void Intake_Out();
    void Intake_Off();
    void Intake_Up();
    void Intake_Down();

    void Shooter_Encoder();
    void Shooter_Off();
    double Get_Distance();
};
#endif