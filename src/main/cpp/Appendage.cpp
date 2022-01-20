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
    rev::SparkMaxRelativeEncoder s_Shooter_Encoder =  m_Shooter->GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42);
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
void Appendage::Shooter_Encoder(){
    double output, target = 3000, current = s_Shooter_Encoder->GetVelocity(), err = target - current, kP = 0.01;
    output = err * kP;
    m_Shooter -> Set(output);
}

/*
 * Turns off
 */
void Appendage::Shooter_Off()
{
    m_Shooter->Set(0);
}

/*
 * Uses camera to measure distance away from goal
 */
double Appendage::Get_Distance()
{
    double distance,
        height1, height2,                                 // height1 is the height of the goal, height2 is the height of the robot,
        angle1, angle2;                                   // angle1 is the smalled angle, andgle2 is the bigger angle

    height1 = 1, height2 = 0.5, angle1 = 15, angle2 = 35; // height are in meters, angles are in degrees
    distance = (height1 - height2) / tan(angle1 + angle2);
    return distance;
}

