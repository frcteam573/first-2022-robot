#include "Appendage.h"

using namespace std;

Appendage::Appendage()
{
    // Define motors, sensors, and pneumatics here
    int m_IntakeId = 3;
    int m_ShooterId = 6;
    int m_SusanId = 13;
    int m_HoodId = 12;

    int p_IntakeId_a = 14;
    int p_IntakeId_b = 15;

    m_Intake = new rev::CANSparkMax{m_IntakeId, rev::CANSparkMax::MotorType::kBrushless};
    m_Shooter = new rev::CANSparkMax{m_ShooterId, rev::CANSparkMax::MotorType::kBrushless};
    m_Susan = new rev::CANSparkMax{m_SusanId, rev::CANSparkMax::MotorType::kBrushless};
    m_Hood = new rev::CANSparkMax{m_HoodId, rev::CANSparkMax::MotorType::kBrushless};

    p_Intake = new frc::DoubleSolenoid{frc::PneumaticsModuleType::REVPH, p_IntakeId_a, p_IntakeId_b};

    // CANEncoder was deprecated as of 2022
    s_Shooter_Encoder = new rev::SparkMaxRelativeEncoder{m_Shooter->GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,42)};
    s_Susan_Encoder = new rev::SparkMaxRelativeEncoder{m_Susan->GetEncoder(rev::SparkMaxRelativeEncoder::EncoderType::kHallSensor, 4096)};
    s_Hood_Encoder = new rev::SparkMaxRelativeEncoder{m_Hood->GetEncoder(rev::SparkMaxRelativeEncoder::EncoderType::kHallSensor, 4096)};
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
 * Turns off the intake
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
    
    //  The two lines below are needed for prototype testing. They allow for control of the system from the Driver Station. 
    //  When uncommented the line below these must be commented out.
    /*double output, target = frc::SmartDashboard::GetNumber("Target",3000), current = s_Shooter_Encoder->GetVelocity(), err = target - current, kP = frc::SmartDashboard::GetNumber("kP",0.01);
    //frc::SmartDashboard::PutNumber("Current",current);*/
    
    double output, target = 3000, current = s_Shooter_Encoder->GetVelocity(), err = target - current, kP = 0.01;
    
    
    output = err * kP;
    m_Shooter -> Set(output);
}

/*
 * Turns off the shooter
 */
void Appendage::Shooter_Off()
{
    m_Shooter->Set(0);
}

/*
 * Uses camera to measure distance away from goal
 */
double Appendage::Get_Distance(double camera_y)
{
    double distance,
        heightGoal, heightBot,
        angleSmall, angleBig = camera_y;

    // height are in meters, angles are in degrees
    heightGoal = 1, heightBot = 0.5, angleSmall = 15, angleBig;
    distance = (heightGoal - heightBot) / tan(angleSmall + angleBig);
    return distance;
}

/*
 * Moves Turret
 */
bool Appendage::Rotate(double camera_exists, double camera_x, bool direction)
{
    double error,
        k = 0.3,
        output,
        currEnc, maxEnc = 4000, minEnc = 1000;

    if (camera_exists <= 0)
    {
        currEnc = s_Susan_Encoder->GetPosition();
        if (currEnc > maxEnc)
        {
            m_Susan->Set(-1);
        }
        else if (currEnc < minEnc)
        {
            m_Susan->Set(1);
        }
        else if (currEnc >= minEnc && currEnc <= maxEnc)
        {
            if (direction)
            {
                m_Susan->Set(1);
            }
            else
            {
                m_Susan->Set(-1);
            }
        }
    }

    error = 0 - camera_x;
    output = k * error;
    m_Susan->Set(output);

    return output;
}

double Appendage::Articulate(double distance){
    double setpoint = distance,
        curr,
        error,
        kP = 0.3,
        output;

    curr = s_Hood_Encoder->GetPosition();
    error = setpoint - curr;
    output = error * kP;
    m_Hood -> Set(output);

    return output;
}
