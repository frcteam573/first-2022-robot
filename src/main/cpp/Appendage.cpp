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
    //int m_HoodId = 12;

    int p_IntakeId_a = 6;
    int p_IntakeId_b = 7;
    int p_Hood_a = 2;
    int p_Hood_b = 3;

    int s_LightGateId = 1;
    int s_LightGate2Id = 2;

    m_Intake1 = new rev::CANSparkMax{m_IntakeId1, rev::CANSparkMax::MotorType::kBrushless};
    m_Intake2 = new rev::CANSparkMax{m_IntakeId2, rev::CANSparkMax::MotorType::kBrushless};
    m_Shooter1 = new rev::CANSparkMax{m_ShooterId1, rev::CANSparkMax::MotorType::kBrushless};
    m_Shooter2 = new rev::CANSparkMax{m_ShooterId2, rev::CANSparkMax::MotorType::kBrushless};
    
    m_Intake1 -> SetInverted(true);
    m_Intake2 -> SetInverted(true);
    m_Feeder = new rev::CANSparkMax{m_FeederId, rev::CANSparkMax::MotorType::kBrushless};
    m_Susan = new rev::CANSparkMax{m_SusanId, rev::CANSparkMax::MotorType::kBrushless};
    p_Hood = new frc::DoubleSolenoid{frc::PneumaticsModuleType::REVPH, p_Hood_a, p_Hood_b};

    m_colorSensor = new rev::ColorSensorV3(frc::I2C::Port::kOnboard);
    m_colorMatcher = new rev::ColorMatch;

    p_Intake = new frc::DoubleSolenoid{frc::PneumaticsModuleType::REVPH, p_IntakeId_a, p_IntakeId_b};

    // CANEncoder was deprecated as of 2022
    
    s_Shooter_Encoder = new rev::SparkMaxRelativeEncoder{m_Shooter1->GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,42)};
    s_Susan_Encoder = new rev::SparkMaxRelativeEncoder{m_Susan->GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)};
    s_LightGate = new frc::DigitalInput(s_LightGateId);
    s_LightGate2 = new frc::DigitalInput(s_LightGate2Id);
    
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
    static double shooter_p_in = 0.00007;
    static double shooter_target_in = 1000;
    static double feed_roller_speed = 0.99;
    static double prefeed_roller_speed = 0.99;
    static double turret_speed = 0.85;
    static double intake_speed = 0.75;
    static double k_turret_cam = 0.03;
    static double k_turret_enc = 0.025;
    static double turret_max_enc = 335;
    static double turret_min_enc = -335;
    static double turret_enc_deadzone = 1;
    static double turret_cam_deadzone = 1;
    static double turret_shooter_deadzone = 150;
    static double feedfor = 0.2;
   
  frc::SmartDashboard::PutNumber("Shooter P In", shooter_p_in);
  frc::SmartDashboard::PutNumber("Shooter Feed Forward In", feedfor);
  frc::SmartDashboard::PutNumber("Shooter Target In", shooter_target_in);
  //frc::SmartDashboard::PutNumber("Feed Speed", feed_roller_speed);
  //frc::SmartDashboard::PutNumber("PreFeed Speed", prefeed_roller_speed);
  //frc::SmartDashboard::PutNumber("Turret Speed", turret_speed);
  //frc::SmartDashboard::PutNumber("Intake Speed", intake_speed);
  //frc::SmartDashboard::PutNumber("Turret Camera P", k_turret_cam);
  //frc::SmartDashboard::PutNumber("Turret Enconder P", k_turret_enc);
  //frc::SmartDashboard::PutNumber("Turret Max Encoder", turret_max_enc);
  //frc::SmartDashboard::PutNumber("Turret Min Encoder", turret_min_enc);
  //frc::SmartDashboard::PutNumber("Turret Enc Deadzone", turret_enc_deadzone);
  frc::SmartDashboard::PutNumber("Turret Cam Deadzone", turret_cam_deadzone);
  frc::SmartDashboard::PutNumber("Shooter Deadzone", turret_shooter_deadzone);
   
  shooterout_old = 0;
}
/*
 * Allows robot to Intake Balls
 */
bool Appendage::Intake_In(char color_in){
    double intakespeed = .75;//frc::SmartDashboard::GetNumber("Intake Speed", 0.99);
    m_Intake1->Set(intakespeed);

    bool output = false;

  //  bool output = s_LightGate->Get();
    
    if (color_in == 'W'){
        output = true;
    }
    return output;
}

/*
 * Allows robot to Intake Balls (Reverse)
 */
void Appendage::Intake_Out()
{
    double intakespeed = 0.75;//frc::SmartDashboard::GetNumber("Intake Speed", 0.99);
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
    double prefeedspeed = 0.75;//frc::SmartDashboard::GetNumber("PreFeed Speed", 0.99);
    m_Intake2->Set(prefeedspeed);
}

/*
 * Allows robot to Intake Balls (Reverse)
 */
void Appendage::Intake2_Out()
{
    double prefeedspeed = 0.75; //frc::SmartDashboard::GetNumber("PreFeed Speed", 0.99);
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
    double feedspeed = 0.99; //frc::SmartDashboard::GetNumber("Feed Speed", 0.99);
    m_Feeder->Set(feedspeed);
}

/*
 * Allows robot to Intake Balls (Reverse)
 */
void Appendage::Feeder_Out()
{
    double feedspeed = 0.99; //frc::SmartDashboard::GetNumber("Feed Speed", 0.99);
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
    p_Intake->Set(frc::DoubleSolenoid::Value::kReverse);
}

/*
 * Allows robots intake to go down
 */
void Appendage::Intake_Down()
{
    p_Intake->Set(frc::DoubleSolenoid::Value::kForward);
}

/*
 * Allows shooter to position when shooting
 */
bool Appendage::Shooter_Encoder(){
     
    
    double current = s_Shooter_Encoder->GetVelocity(); // Function returns RPM
    //current = abs(current);
    // Read in value from dashboard
    double shooter_p_in = frc::SmartDashboard::GetNumber("Shooter P In", 0.00007);
    double shooter_target_in = frc::SmartDashboard::GetNumber("Shooter Target In", 3000);
    //double shooter_f_in = frc::SmartDashboard::GetNumber("Shooter Feed Forward In", 0.2);

    double kP = shooter_p_in;
    double target = shooter_target_in;

    double gear_ratio = 2/1; // Gear ratio between shooter motor encoder and shooter wheel

    current = current * gear_ratio;

    double err = target - current;

    //double shooter_f_in = 0.0985495 * target / 1000 + 0.019278;
    double shooter_f_in = 0.00009 * target + 0.0072;
    

    double output = (err * kP) + shooter_f_in;  

    bool atspeed = false;

    if (abs (err) < 250){
        atspeed = true;
    }

    output = this->Remap_Val(output, 0.99);

    if(abs(output - shooterout_old)>0.3){
       output =  0.3*(output - shooterout_old)/abs(output - shooterout_old) + shooterout_old;
    }

    m_Shooter1 -> Set(output);
    m_Shooter2 -> Set(-output);

    shooterout_old = output;

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
    double kP = 0.00007;
    distance = distance + (trim * 6); // Every trim value will be 6 inches futher / closer
    double target;
    if(distance <= 70){ // 70 in is hood up down cut off
        target = 13.1389*distance+3269.23;
    }
    else{
        target = 13.1389*distance+3269.23;
    }

    double gear_ratio = 2/1; // Gear ratio between shooter motor encoder and shooter wheel

    current = current * gear_ratio;

    double err = target - current;

    //double shooter_f_in = 0.0985495 * target / 1000 + 0.019278;

    double shooter_f_in = 0.00009 * target + 0.0072;

    double output = (err * kP) + shooter_f_in;  

    bool atspeed = false;

    double i = frc::SmartDashboard::GetNumber("Shooter Deadzone", 150);
    if (abs (err) < i){
        atspeed = true;
    }

    output = this->Remap_Val(output, 0.99);

     if(abs(output - shooterout_old)>0.3){
       output =  0.3*(output - shooterout_old)/abs(output - shooterout_old) + shooterout_old;
    }

    m_Shooter1 -> Set(output);
    m_Shooter2 -> Set(-output);

    shooterout_old = output;

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
    shooterout_old = 0;
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
    heightGoal = 102.8125, heightBot = 41.5, angleMount = 34; // From CAD not measured on robot
    double PI = 3.14159265;
    double rads = (angleMount + angleCam) * PI / 180.0;
    distance = (heightGoal - heightBot) / tan(rads);
    return distance;
}

/*
 * Moves Turret
 */
std::tuple<bool, bool> Appendage::Rotate(double camera_exists, double camera_x, bool direction, bool fixedgoal, bool endgame, bool kAuto)
{

    double error,
        k = 0.03,
        k_fixedpos = 0.025,
        output = 0,
        currEnc, maxEnc = 335, minEnc = -335, turret_enc_deadzone = 1, turret_cam_deadzone = 1;

    bool align = false;

     /*k = frc::SmartDashboard::GetNumber("Turret Camera P", 0.03);
     k_fixedpos = frc::SmartDashboard::GetNumber("Turret Enconder P", 0.025);
     maxEnc = frc::SmartDashboard::GetNumber("Turret Max Encoder", 335);
     minEnc = frc::SmartDashboard::GetNumber("Turret Min Encoder", -335);
     turret_enc_deadzone = frc::SmartDashboard::GetNumber("Turret Enc Deadzone", 1);
     turret_cam_deadzone = frc::SmartDashboard::GetNumber("Turret Cam Deadzone", 1);
    */
// Fixed positiion
    if (fixedgoal){
        error = 0 - s_Susan_Encoder->GetPosition();
        output = k_fixedpos * error;

        if(abs(error)<turret_enc_deadzone){
            output = 0;
            align = true;
        }
    } 
    else if (kAuto){
        error = 15 - s_Susan_Encoder->GetPosition();
        output = k_fixedpos * error;

        if(abs(error)<turret_enc_deadzone){
            output = 0;
            align = true;
        }
    } 

    else if(endgame){ //Endgame
        double currpos = s_Susan_Encoder->GetPosition();

        double setpoint;

        if(currpos >= 0){
            setpoint = 335;     // Need to update with 180 degree encoder value
        }
        else{
            setpoint = -335;
        }

        error = setpoint - currpos; 
        output = k_fixedpos * error;

        if(abs(error)<turret_enc_deadzone){
            output = 0;
            align = true;
        }
    }

    // Camera Tracking
    else{
        

        if(camera_exists == 1){ 
            error = 0 - camera_x;
            output = k * error;

            if(abs(error)<turret_cam_deadzone){   // Need to set range when testing.
                align = true;
                output = 0;
            }
        }
        else if(camera_exists == 2){ // Just incase camera stream is broken
            output = 0;
        }
        else {   // Camera sees no target

            /*// only use if we trust belt won't skip
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
            */
            output = 0;

        }

    }

    if(output >= 0){
        direction = true;
    }
    else{
        direction = false;
    }

    double turretspeed = 0.85; // frc::SmartDashboard::GetNumber("Turret Speed", 0.5);
    
    output = Remap_Val(output,turretspeed);

    m_Susan->Set(output);   

    return std::make_tuple(align,direction);
}

//Lazy Susan Testing

    void Appendage::Rotate_left()
{
    double turretspeed =  0.85; //frc::SmartDashboard::GetNumber("Turret Speed", 0.99);
    m_Susan->Set(-turretspeed);
}

    void Appendage::Rotate_right()
{
    double turretspeed = 0.85; //frc::SmartDashboard::GetNumber("Turret Speed", 0.99);
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

    if (distance > 70){
        p_Hood->Set(frc::DoubleSolenoid::Value::kReverse);
    }

    else 
        {p_Hood->Set(frc::DoubleSolenoid::Value::kForward);}

}

// color sensor
void Appendage::controlpanel_colorsense_init(){

ct=0;
  //Any updates here also have to be done in Appendage.h
  static constexpr frc::Color kBlueTarget = frc::Color(0.1433, 0.427, 0.429);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kWhiteTarget = frc::Color(0.365, 0.464, 0.169);

  m_colorMatcher->AddColorMatch(kBlueTarget);
  m_colorMatcher->AddColorMatch(kRedTarget);
  //m_colorMatcher->AddColorMatch(kWhiteTarget);

}

  char Appendage::controlpanel_colorsense_periodic(){
    // Fucntion spins contorl panel to specified color recieved from driver station
 
     frc::Color detectedColor = m_colorSensor->GetColor();
    //frc::SmartDashboard::PutNumber("OutColor",m_colorSensor->GetRawColor());
    rev::ColorSensorV3::RawColor colorraw = m_colorSensor -> GetRawColor();
    frc::SmartDashboard::PutNumber("OutColorRed",detectedColor.red);
    frc::SmartDashboard::PutNumber("OutColorBlue",detectedColor.blue);
    frc::SmartDashboard::PutNumber("OutColorGreen",detectedColor.green);
    ct ++; 
    frc::SmartDashboard::PutNumber("Count",ct);

    // Get raw RGB values from color sensor and display on DS
    /*frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    auto encoder_valstr = std::to_string(detectedColor.red);
    frc::SmartDashboard::PutString("DB/String 0",encoder_valstr);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    auto encoder_valstr1 = std::to_string(detectedColor.blue);
    frc::SmartDashboard::PutString("DB/String 1",encoder_valstr1);
    auto encoder_valstr2 = std::to_string(detectedColor.green);
    frc::SmartDashboard::PutString("DB/String 2",encoder_valstr2); */
      
    //Run the color match algorithm on our detected color

      std::string colorString;
      char colorchar;
      double confidence = 0.0;

      frc::Color matchedColor = m_colorMatcher->MatchClosestColor(detectedColor, confidence); // Determine color
      double color_prox = m_colorSensor-> GetProximity();
      frc::SmartDashboard::PutNumber("ColorProx",color_prox);
     if(color_prox>200){
      if (matchedColor == kBlueTarget) {
        colorString = "B";
         colorchar =  'B';
      } else if (matchedColor == kRedTarget) {
        colorString = "R";
        colorchar =  'R';

      }  else {
        colorString = "B";
        colorchar =  'B';

      }
     }
     else{
        colorString = "W";
        colorchar =  'W';
     }



      //Display what color is seen on DS
      frc::SmartDashboard::PutString("Current Color", colorString);

      return colorchar;
}

int Appendage::BallCounter(char colorIn){
    int BallCnt = 0;
    
    if (!s_LightGate->Get()){
        BallCnt+=1;
    }
    if (!s_LightGate2->Get()){
        BallCnt+=1;
    }
    if (colorIn != 'W'){
        BallCnt+=1;
    }
    
    return BallCnt;
}

/*Appendage Dashboard*/
    void Appendage::dashboard(){
        frc::SmartDashboard::PutNumber("Shooter Enc", (s_Shooter_Encoder -> GetVelocity())*2);
        frc::SmartDashboard::PutNumber("Susan Enc", s_Susan_Encoder -> GetPosition());
        frc::SmartDashboard::PutBoolean("LightGate",s_LightGate->Get());
    }
