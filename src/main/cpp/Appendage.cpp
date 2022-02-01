#include "Appendage.h"

using namespace std;

Appendage::Appendage()
{
    // Define motors, sensors, and pneumatics here
    int m_IntakeId1 = 3;
    int m_IntakeId2 = 11;
    int m_ShooterId1 = 14;
    int m_ShooterId2 = 2;
    int m_FeederId = 6;
    int m_SusanId = 13;
    int m_HoodId = 12;

    int p_IntakeId_a = 14;
    int p_IntakeId_b = 15;

    m_Intake1 = new rev::CANSparkMax{m_IntakeId1, rev::CANSparkMax::MotorType::kBrushless};
    m_Intake2 = new rev::CANSparkMax{m_IntakeId2, rev::CANSparkMax::MotorType::kBrushless};
    m_Shooter1 = new rev::CANSparkMax{m_ShooterId1, rev::CANSparkMax::MotorType::kBrushless};
    m_Shooter2 = new rev::CANSparkMax{m_ShooterId2, rev::CANSparkMax::MotorType::kBrushless};

    m_Shooter1 -> SetInverted(true);
    m_Shooter2 -> SetInverted(true);
    m_Feeder = new rev::CANSparkMax{m_FeederId, rev::CANSparkMax::MotorType::kBrushless};
    m_Susan = new rev::CANSparkMax{m_SusanId, rev::CANSparkMax::MotorType::kBrushless};
    m_Hood = new rev::CANSparkMax{m_HoodId, rev::CANSparkMax::MotorType::kBrushless};

    p_Intake = new frc::DoubleSolenoid{frc::PneumaticsModuleType::REVPH, p_IntakeId_a, p_IntakeId_b};

    // CANEncoder was deprecated as of 2022
    
    s_Shooter_Encoder = new rev::SparkMaxRelativeEncoder{m_Shooter1->GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,42)};
    s_Susan_Encoder = new rev::SparkMaxRelativeEncoder{m_Susan->GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)};
    s_Hood_Encoder = new rev::SparkMaxRelativeEncoder{m_Hood->GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)};

    
}

/*
 * Remaps a number
 */
double Appendage::Remap_Val(double i, double threshold)
{
    if (abs(i) > threshold)
    {
        i = i/abs(i) * threshold;
    }

    return i;
}

/* Creates Dashboard Inputs Needed for Appendage */
void Appendage::DashboardCreate(){
    
// Dashboard input inital decs.
    static double shooter_p_in = 1;
    static double shooter_target_in = 3000;
    static double shooter_f_in = 0.2;

  frc::SmartDashboard::PutNumber("Shooter P In", shooter_p_in);
  frc::SmartDashboard::PutNumber("Shooter Target In", shooter_target_in);
  frc::SmartDashboard::PutNumber("Shooter Feed Forward In", shooter_f_in);
}
/*
 * Allows robot to Intake Balls
 */
void Appendage::Intake1_In()
{
    m_Intake1->Set(1);
}

/*
 * Allows robot to Intake Balls (Reverse)
 */
void Appendage::Intake1_Out()
{
    m_Intake1->SetInverted(1);
}

/*
 * Turns off the intake
 */
void Appendage::Intake1_Off()
{
    m_Intake1->Set(0);
}
/*
 * Allows robot to Intake Balls
 */
void Appendage::Intake2_In()
{
    m_Intake2->Set(-1);
}

/*
 * Allows robot to Intake Balls (Reverse)
 */
void Appendage::Intake2_Out()
{
    m_Intake2->SetInverted(-1);
}

/*
 * Turns off the intake
 */
void Appendage::Intake2_Off()
{
    m_Intake2->Set(0);
}
/*
 * Allows robot to Intake Balls
 */
void Appendage::Feeder_In()
{
    m_Feeder->Set(1);
}

/*
 * Allows robot to Intake Balls (Reverse)
 */
void Appendage::Feeder_Out()
{
    m_Feeder->SetInverted(1);
}

/*
 * Turns off the intake
 */
void Appendage::Feeder_Off()
{
    m_Feeder->Set(0);
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
bool Appendage::Shooter_Encoder(){
     
    
    double current = s_Shooter_Encoder->GetVelocity(); // Function returns RPM
    
    // Read in value from dashboard
    double shooter_p_in = frc::SmartDashboard::GetNumber("Shooter P In", 1);
    double shooter_target_in = frc::SmartDashboard::GetNumber("Shooter Target In", 3000);
    double shooter_f_in = frc::SmartDashboard::GetNumber("Shooter Feed Forward In", 0.2);

    double kP = shooter_p_in;
    double target = shooter_target_in;

    double gear_ratio = 1/1.5; // Gear ratio between shooter motor encoder and shooter wheel

    current = current * gear_ratio;

    double err = target - current;

    double output = (err * kP) + shooter_f_in;  

    bool atspeed = false;

    if (abs (err) < 250){
        atspeed = true;
    }

    output = this->Remap_Val(output, 0.99);

    m_Shooter1 -> Set(output);
    m_Shooter2 -> Set(output);

    //Output to dash for testing
    frc::SmartDashboard::PutNumber("Shooter Target Out", shooter_target_in);
    frc::SmartDashboard::PutNumber("Shooter Current", current);
    frc::SmartDashboard::PutNumber("Shooter Error", err);
    frc::SmartDashboard::PutNumber("Shooter P Out", shooter_p_in);
    frc::SmartDashboard::PutNumber("Shooter Output", output);
    return atspeed;
}

/*
 * Turns off the shooter
 */
void Appendage::Shooter_Off()
{
    m_Shooter1->Set(0);
    m_Shooter2->Set(0);
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
    heightGoal = 1, heightBot = 0.5, angleSmall = 15, angleBig = 30;
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
        output = 0,
        currEnc, maxEnc = 4000, minEnc = 1000;

    if (camera_exists <= 0)
    {
        currEnc = s_Susan_Encoder->GetPosition();
        if (currEnc > maxEnc)
        {
            output = -1;
        }
        else if (currEnc < minEnc)
        {
            output = 1;
        }
        else if (currEnc >= minEnc && currEnc <= maxEnc)
        {
            if (direction)
            {
                output = 1;
            }
            else
            {
                output = -1;
            }
        }
    }
    else{

        error = 0 - camera_x;
        output = k * error;

    }

    if(output >= 0){
        direction = true;
    }
    else{
        direction = false;
    }

    m_Susan->Set(output);    
    return direction;
}

/*
 * Turns off the turret Rotation
 */
void Appendage::Rotate_Off()
{
    m_Susan->Set(0);
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


