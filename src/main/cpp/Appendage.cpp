#include "Appendage.h"

using namespace std;

Appendage::Appendage()
{
    int m_IntakeId = 3;
    int p_IntakeId_a = 14;
    int p_IntakeId_b = 15;
    int m_ShooterId = 6;
    // Define motors, sensors, and pneumatics here
    m_Intake = new rev::CANSparkMax{m_IntakeId, rev::CANSparkMax::MotorType::kBrushless}; // actually 1
    p_Intake = new frc::DoubleSolenoid{frc::PneumaticsModuleType::REVPH, p_IntakeId_a, p_IntakeId_b};
    m_Shooter = new rev::CANSparkMax{m_ShooterId, rev::CANSparkMax::MotorType::kBrushless};
}

/*
 * Allows robot to Intake Balls
 */
void Appendage::Intake_In()
{
    m_Intake->Set(1);
}

/*
 * Allows robot to Intake Balls (Reverse)
 */
void Appendage::Intake_Out()
{
    m_Intake->SetInverted(1);
}

/*
 * Turns off
 */
void Appendage::Intake_Off()
{
    m_Intake->Set(0);
}

/*
 * Allows robots intake to go up
 */
void Appendage::Intake_Up()
{
    p_Intake->Set(frc::DoubleSolenoid::Value::kForward);
}

/*
 * Allows robots intake to go down
 */
void Appendage::Intake_Down()
{
    p_Intake->Set(frc::DoubleSolenoid::Value::kReverse);
}

/*
 * Allows shooter to position when shooting
 */
int Appendage::Shooter_Encoder(){
    double output;
    return output;
}