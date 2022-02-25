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

    int p_IntakeId_a = 6;
    int p_IntakeId_b = 7;
    int p_Hood_a = 4;
    int p_Hood_b = 5;
    int s_LightGateId = 0;

    m_Intake1 = new rev::CANSparkMax{m_IntakeId1, rev::CANSparkMax::MotorType::kBrushless};
    m_Intake2 = new rev::CANSparkMax{m_IntakeId2, rev::CANSparkMax::MotorType::kBrushless};
    m_Shooter1 = new rev::CANSparkMax{m_ShooterId1, rev::CANSparkMax::MotorType::kBrushless};
    m_Shooter2 = new rev::CANSparkMax{m_ShooterId2, rev::CANSparkMax::MotorType::kBrushless};

    m_Intake2 -> SetInverted(true);
    m_Feeder = new rev::CANSparkMax{m_FeederId, rev::CANSparkMax::MotorType::kBrushless};
    m_Susan = new rev::CANSparkMax{m_SusanId, rev::CANSparkMax::MotorType::kBrushless};
    p_Hood = new frc::DoubleSolenoid{frc::PneumaticsModuleType::REVPH, p_Hood_a, p_Hood_b};

    p_Intake = new frc::DoubleSolenoid{frc::PneumaticsModuleType::REVPH, p_IntakeId_a, p_IntakeId_b};

    // CANEncoder was deprecated as of 2022
    
    s_Shooter_Encoder = new rev::SparkMaxRelativeEncoder{m_Shooter1->GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,42)};
    s_Susan_Encoder = new rev::SparkMaxRelativeEncoder{m_Susan->GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)};
    s_LightGate = new frc::DigitalInput(s_LightGateId);
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
    static double shooter_p_in = 0.00025;
    static double shooter_target_in = 1000;
    static double feed_roller_speed = 0.99;
    static double prefeed_roller_speed = 0.99;
    static double turret_speed = 0.99;
    static double intake_speed = 0.99;
   
  frc::SmartDashboard::PutNumber("Shooter P In", shooter_p_in);
  frc::SmartDashboard::PutNumber("Shooter Target In", shooter_target_in);
  frc::SmartDashboard::PutNumber("Feed Speed", feed_roller_speed);
  frc::SmartDashboard::PutNumber("PreFeed Speed", prefeed_roller_speed);
  frc::SmartDashboard::PutNumber("Turret Speed", turret_speed);
  frc::SmartDashboard::PutNumber("Intake Speed", intake_speed);
  
}
/*
 * Allows robot to Intake Balls
 */
bool Appendage::Intake_In()
{
    double intakespeed = frc::SmartDashboard::GetNumber("Intake Speed", 0.99);
    m_Intake1->Set(intakespeed);

    return s_LightGate -> Get();
}

/*
 * Allows robot to Intake Balls (Reverse)
 */
void Appendage::Intake_Out()
{
    double intakespeed = frc::SmartDashboard::GetNumber("Intake Speed", 0.99);
    m_Intake1->Set(-intakespeed);
}

/*
 * Turns off the intake
 */
void Appendage::Intake_Off()
{
    m_Intake1->Set(0);
}

/*
 * Allows robot to Intake Balls
 */
void Appendage::Intake2_In()
{
    double prefeedspeed = frc::SmartDashboard::GetNumber("PreFeed Speed", 0.99);
    m_Intake2->Set(prefeedspeed);
}

/*
 * Allows robot to Intake Balls (Reverse)
 */
void Appendage::Intake2_Out()
{
    double prefeedspeed = frc::SmartDashboard::GetNumber("PreFeed Speed", 0.99);
    m_Intake2->Set(-prefeedspeed);
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
    double feedspeed = frc::SmartDashboard::GetNumber("Feed Speed", 0.99);
    m_Feeder->Set(feedspeed);
}

/*
 * Allows robot to Intake Balls (Reverse)
 */
void Appendage::Feeder_Out()
{
    double feedspeed = frc::SmartDashboard::GetNumber("Feed Speed", 0.99);
    m_Feeder->Set(-feedspeed);
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
    //current = abs(current);
    // Read in value from dashboard
    double shooter_p_in = frc::SmartDashboard::GetNumber("Shooter P In", 0.00025);
    double shooter_target_in = frc::SmartDashboard::GetNumber("Shooter Target In", 3000);
    //double shooter_f_in = frc::SmartDashboard::GetNumber("Shooter Feed Forward In", 0.2);

    double kP = shooter_p_in;
    double target = shooter_target_in;

    double gear_ratio = 1/1; // Gear ratio between shooter motor encoder and shooter wheel

    current = current * gear_ratio;

    double err = target - current;

    double shooter_f_in = 0.0985495 * target / 1000 + 0.019278;

    double output = (err * kP) + shooter_f_in;  

    bool atspeed = false;

    if (abs (err) < 250){
        atspeed = true;
    }

    output = this->Remap_Val(output, 0.99);

    m_Shooter1 -> Set(output);
    m_Shooter2 -> Set(output);

    //Output to dash for testing
    //frc::SmartDashboard::PutNumber("Shooter Target Out", shooter_target_in);
    //frc::SmartDashboard::PutNumber("Shooter Current", current);
    //frc::SmartDashboard::PutNumber("Shooter Error", err);
    //frc::SmartDashboard::PutNumber("Shooter P Out", shooter_p_in);
    //frc::SmartDashboard::PutNumber("Shooter Output", output);
    //frc::SmartDashboard::PutNumber("F Output", shooter_f_in);
    return atspeed;
}

bool Appendage::Shooter_Encoder_distance(double distance, double trim){
     
    
    double current = s_Shooter_Encoder->GetVelocity(); // Function returns RPM
    //current = abs(current);
    double kP = 0.00025;
    distance = distance + (trim * 6); // Every trim value will be 6 inches futher / closer
     
    double target = distance; // Will need to add some math to convert distance to target speed

    double gear_ratio = 1/1; // Gear ratio between shooter motor encoder and shooter wheel

    current = current * gear_ratio;

    double err = target - current;

    double shooter_f_in = 0.0985495 * target / 1000 + 0.019278;

    double output = (err * kP) + shooter_f_in;  

    bool atspeed = false;

    if (abs (err) < 250){
        atspeed = true;
    }

    output = this->Remap_Val(output, 0.99);

    m_Shooter1 -> Set(output);
    m_Shooter2 -> Set(output);

    //Output to dash for testing
    //frc::SmartDashboard::PutNumber("Shooter Current", current);
    //frc::SmartDashboard::PutNumber("Shooter Error", err);
    //frc::SmartDashboard::PutNumber("Shooter Output", output);
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
        angleMount, angleCam = camera_y;

    // height are in (inches), angles are in degrees
    heightGoal = 102.8125, heightBot = 41.5, angleMount = 39; // From CAD not measured on robot
    double PI = 3.14159265;
    double rads = (angleMount + angleCam) * PI / 180.0;
    distance = (heightGoal - heightBot) / tan(rads);
    return distance;
}

/*
 * Moves Turret
 */
std::tuple<bool, bool> Appendage::Rotate(double camera_exists, double camera_x, bool direction, bool fixedgoal, bool endgame)
{
    double error,
        k = 0.3,
        k_fixedpos = 0.2,
        output = 0,
        currEnc, maxEnc = 4000, minEnc = 1000;

    bool align = false;

// Fixed positiions
    if (fixedgoal){
        error = 0 - s_Susan_Encoder->GetPosition();
        output = k_fixedpos * error;

        if(abs(error)<20){
            output = 0;
            align = true;
        }
    } 
    else if(endgame){
        error = 180 - s_Susan_Encoder->GetPosition(); // Need to update with 180 degree encoder value
        output = k_fixedpos * error;

        if(abs(error)<20){
            output = 0;
            align = true;
        }
    }

    // Camera Tracking
    else{

        if (camera_exists == 0){ // Camera doesn't see target
            currEnc = s_Susan_Encoder->GetPosition();
            if (currEnc > maxEnc){
                output = -1;            // Need to confirm this is right direction and speed
            }
            else if (currEnc < minEnc){
                output = 1;             // Need to confirm this is right direction and speed
            }
            else if (currEnc >= minEnc && currEnc <= maxEnc){
                if (direction){
                    output = 1;     // Need to confirm this is right direction and speed
                }
                else{
                    output = -1;    // Need to confirm this is right direction and speed
                }
            }
        }
        else{   // Camera sees target

            error = 0 - camera_x;
            output = k * error;

            if(abs(error)<2){   // Need to set range when testing.
                align = true;
            }

        }

    }

    if(output >= 0){
        direction = true;
    }
    else{
        direction = false;
    }

    m_Susan->Set(output);   

    return std::make_tuple(align,direction);
}

//Lazy Susan Testing

    void Appendage::Rotate_left()
{
    double turretspeed = frc::SmartDashboard::GetNumber("Turret Speed", 0.99);
    m_Susan->Set(-turretspeed);
}

    void Appendage::Rotate_right()
{
    double turretspeed = frc::SmartDashboard::GetNumber("Turret Speed", 0.99);
    m_Susan->Set(turretspeed);
}

/*
 * Turns off the turret Rotation
 */
void Appendage::Rotate_Off()
{
    m_Susan->Set(0);
}

void Appendage::Articulate(double distance){

    if (distance > 120){
        p_Hood->Set(frc::DoubleSolenoid::Value::kForward);
    }

    else 
        {p_Hood->Set(frc::DoubleSolenoid::Value::kReverse);}

}

/*Appendage Dashboard*/
    void Appendage::dashboard(){
        frc::SmartDashboard::PutNumber("Shooter Enc", s_Shooter_Encoder -> GetVelocity());
        frc::SmartDashboard::PutNumber("Susan Enc", s_Susan_Encoder -> GetPosition());
        frc::SmartDashboard::PutBoolean("LightGate",s_LightGate->Get());
    }
