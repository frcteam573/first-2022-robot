#include "Drive.h"
#include "rev/CANSparkMax.h"
#include "frc/DoubleSolenoid.h"
#include "frc/ADXRS450_Gyro.h"

using namespace std;

Drive::Drive(){
    int leftdriveID = 9;
    int leftdriveID2 = 10;
    int rightdriveID = 7;
    int rightdriveID2 = 8;
    int leftclimbID = 4;
    int rightclimbID = 5;   
    int climberlockIDa = 0;
    int climberlockIDb = 1;
    int climber_tiltIDa = 2;
    int climber_tiltIDb = 3;


// Define motors, sensors, and pneumatics here
    // Drive motors, sensors, and pneumatics

        m_leftdrive = new rev::CANSparkMax{leftdriveID, rev::CANSparkMax::MotorType::kBrushless}; //actually 1
        m_leftdrive2 = new rev::CANSparkMax{leftdriveID2, rev::CANSparkMax::MotorType::kBrushless};

        m_rightdrive = new rev::CANSparkMax{rightdriveID, rev::CANSparkMax::MotorType::kBrushless};
        m_rightdrive2 = new rev::CANSparkMax{rightdriveID2, rev::CANSparkMax::MotorType::kBrushless};
        m_rightdrive -> SetInverted(true);
        m_rightdrive2 -> SetInverted(true);
        s_gyro = new frc::ADXRS450_Gyro(frc::SPI::Port::kOnboardCS0);

    // Climb motors, sensors, and pneumatics
    
        m_leftclimb = new rev::CANSparkMax{leftclimbID, rev::CANSparkMax::MotorType::kBrushless};
        m_rightclimb = new rev::CANSparkMax{rightclimbID, rev::CANSparkMax::MotorType::kBrushless};
        m_leftclimb -> SetInverted(true);  
 

        p_climberlock = new frc::DoubleSolenoid{frc::PneumaticsModuleType::REVPH, climberlockIDb, climberlockIDa};  
        p_climbertilt = new frc::DoubleSolenoid{frc::PneumaticsModuleType::REVPH, climber_tiltIDb, climber_tiltIDa};

        s_leftclimber_enc = new rev::SparkMaxRelativeEncoder{m_leftclimb -> GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,42)};
        s_rightclimber_enc = new rev::SparkMaxRelativeEncoder{m_rightclimb -> GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,42)};

        s_leftdrive_enc = new rev::SparkMaxRelativeEncoder{m_leftdrive -> GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,42)};
        s_rightdrive_enc = new rev::SparkMaxRelativeEncoder{m_rightdrive -> GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,42)};

}


void Drive::DashboardCreate(){
    
// Dashboard input inital decs.
    static double kvelo_in = 3209.6;
    static double kpos_in = 0;
    static double kph_in = -0.0072;

  frc::SmartDashboard::PutNumber("KVelo In", kvelo_in);
  frc::SmartDashboard::PutNumber("KPos_in", kpos_in);
  frc::SmartDashboard::PutNumber("KPH_in", kph_in); 

  frc::SmartDashboard::PutNumber("KP_Gyro", -0.0027); 
  frc::SmartDashboard::PutNumber("KP_DriveDist", -0.07); 

  left_drive_old = 0;
  right_drive_old = 0;
  
}

/* DEADBAND FUNCTION */
/* use to create a deadband on the controls, passing the input and the deadband size */

double Drive::deadband(double input, double deadband_size){
    if (abs(input) < deadband_size){
        input = 0;
    }
    return input;
}

/* JOYSTICK DRIVE */
/* This function provides basic joystick control of the drive base*/
    
    void Drive::Joystick_Drive(double LeftStick, double RightStick){

    double left_out = LeftStick*LeftStick*LeftStick;
    double right_out = RightStick*RightStick*RightStick;
    left_out = deadband(left_out,0.05);
    right_out = deadband(right_out,0.05);

    if(abs(left_out - left_drive_old)>0.5){
       left_out =  0.5*(left_out - left_drive_old)/abs(left_out - left_drive_old) + left_drive_old;
    }

    if(abs(right_out - right_drive_old)>0.5){
       right_out =  0.5*(right_out - right_drive_old)/abs(right_out - right_drive_old) + right_drive_old;
    }

    m_leftdrive -> Set(left_out);
    m_leftdrive2 -> Set(left_out);
    m_rightdrive -> Set(right_out);
    m_rightdrive2 -> Set(right_out);

    left_drive_old = left_out;
    right_drive_old = right_out;
}

//DRIVE STRAIGHT//
        void Drive::drive_straight(bool first, double joystick_y){
       
            if (first)
            {
                s_gyro -> Reset();
            }
            
        double error = 0-s_gyro -> GetAngle();
        double constant = 0.01;

        double turn_out = constant*error; 
        turn_out = Remap_Val (turn_out, 0.7);

           double left_out = joystick_y*joystick_y*joystick_y + turn_out;
           double right_out = joystick_y*joystick_y*joystick_y - turn_out;

        m_leftdrive -> Set(left_out);
        m_leftdrive2 -> Set(left_out);
        m_rightdrive -> Set(right_out);
        m_rightdrive2 -> Set(right_out);

        }

/*
 * Remaps a number
 */
double Drive::Remap_Val(double i, double threshold)
{
    if (abs(i) > threshold)
    {
        i = i/abs(i) * threshold;
    }

    return i;
}

/* CLIMBING */

        int Drive::climber_extend(){

             int output;
            double climboutleft;
            double climboutright;
            if(s_leftclimber_enc->GetPosition() < 160){

                double k_c = 0.01;
                double err = 161 - s_leftclimber_enc->GetPosition();

                climboutleft = Remap_Val(err*k_c,0.99);

            }
            else{climboutleft = 0;}
            

            if(s_rightclimber_enc->GetPosition() > -160 ){

                double k_c = 0.01;
                double err = -161 - s_rightclimber_enc->GetPosition();

                climboutright = Remap_Val(err*k_c,0.99);
            }
           else{climboutright = 0;}

  if (climboutright !=0 ||climboutleft!=0){
     p_climberlock-> Set(frc::DoubleSolenoid::Value::kReverse);

            if (climb_lock > 2){
                m_leftclimb -> Set(climboutleft);
                m_rightclimb -> Set(climboutright);
            }
            else{
                m_leftclimb -> Set(0);
                m_rightclimb -> Set(0);
            }
            
            climb_lock ++;
            }
            
            else{
                output = 1;
                m_leftclimb -> Set(0);
                m_rightclimb -> Set(0);
                p_climberlock-> Set(frc::DoubleSolenoid::Value::kForward);
            }
            return output;

            }

            int Drive::climber_extend_first(){

            int output;
            double climboutleft;
            double climboutright;
            if(s_leftclimber_enc->GetPosition() < 150 ){

                double k_c = 0.01;
                double err = 151 - s_leftclimber_enc->GetPosition();

                climboutleft = Remap_Val(err*k_c,0.99);

            }
            else{climboutleft = 0;}
            

            if(s_rightclimber_enc->GetPosition() > -150 ){

                double k_c = 0.01;
                double err = -151 - s_rightclimber_enc->GetPosition();

                climboutright = Remap_Val(err*k_c,0.99);
            }
           else{climboutright = 0;}

  if (climboutright !=0 ||climboutleft!=0){
     p_climberlock-> Set(frc::DoubleSolenoid::Value::kReverse);

            if (climb_lock > 2){
                m_leftclimb -> Set(climboutleft);
                m_rightclimb -> Set(climboutright);
            }
            else{
                m_leftclimb -> Set(0);
                m_rightclimb -> Set(0);
            }
            
            climb_lock ++;
            }
            
            else{
                output = 1;
                m_leftclimb -> Set(0);
                m_rightclimb -> Set(0);
                p_climberlock-> Set(frc::DoubleSolenoid::Value::kForward);
            }
            return output;

            }


        

        int Drive::climber_retract(){
           
            //unlock climbers and retract
            int output;
            if(s_leftclimber_enc->GetPosition() > 5 ){
            output = 0;
            p_climberlock-> Set(frc::DoubleSolenoid::Value::kReverse);

            if (climb_lock > 2){
                m_leftclimb -> Set(-1);
                m_rightclimb -> Set(1);
            }
            else{
                m_leftclimb -> Set(0);
                m_rightclimb -> Set(0);
            }
            climb_lock ++;
            }
            else{
                output = 1; 
                m_leftclimb -> Set(0);
                m_rightclimb -> Set(0);
                p_climberlock-> Set(frc::DoubleSolenoid::Value::kForward);
            }
            return output;
            
        }

        void Drive::climber_hold(){
           
            //unlock climbers and hold
            p_climberlock -> Set(frc::DoubleSolenoid::Value::kForward);

            climb_lock = 0;
            
                m_leftclimb -> Set(0);
                m_rightclimb -> Set(0);
        }

        void Drive::climber_tiltin(){  
            p_climbertilt -> Set(frc::DoubleSolenoid::Value::kForward);
        }

        void Drive::climber_tiltout(){  
            p_climbertilt -> Set(frc::DoubleSolenoid::Value::kReverse);
        }


    /* ClIMBING AUTON*/
        tuple <bool, bool> Drive::climber_setpoint(string input){
                // Need to update left climb up is pos right climb up is negative 155 is the limit.
           double setpoint, setpoint_output;
           double max, min;

                if(input == "extend"){
                    setpoint = 155;
                    setpoint_output =1;
                    }

                    else{
                        setpoint = 0;
                        setpoint_output=-1;
                    }

                bool output = false;
                bool output_1 = false;
                bool left_inpos = false;
                bool right_inpos = false;
                double left_pos= s_leftclimber_enc -> GetPosition();
                double right_pos = s_rightclimber_enc -> GetPosition();
                
                if (input == "extend"){
                    if (left_pos > setpoint){
                        m_leftclimb->Set(0);
                        left_inpos = true;
                    }
                    else{
                        m_leftclimb->Set(setpoint_output);
                    }
                    if (right_pos > setpoint){
                        m_rightclimb->Set(0);
                        right_inpos = true;
                    }
                    else{
                        m_rightclimb->Set(setpoint_output);
                    }
                }
                else{
                    if (left_pos < setpoint){
                        m_leftclimb->Set(0);
                        left_inpos = true;
                    }
                    else{
                        m_leftclimb->Set(setpoint_output);
                    }
                    if (right_pos < setpoint){
                        m_rightclimb->Set(0);
                        right_inpos = true;
                    }
                    else{
                        m_rightclimb->Set(setpoint_output);
                    }
                }
                
                    if(left_inpos && right_inpos){
                        output = true;
                        p_climberlock-> Set(frc::DoubleSolenoid::Value::kForward);
                    }

                    else{
                        p_climberlock-> Set(frc::DoubleSolenoid::Value::kReverse);
                    }

                    if (s_leftclimber_enc -> GetPosition() > 3000 && s_rightclimber_enc -> GetPosition() > 3000){
                        output_1 = true;
                    }
                        return std::make_tuple(output,output_1);
        }

/* CAMERA INTAKE */

    void Drive::camera_intake(double camera_x, double joystick_y){
       
       double error = 0-camera_x;
       double constant = 0.01;


        double turn_out = constant*error; 
        turn_out = Remap_Val (turn_out, 0.7);

           double left_out = joystick_y*joystick_y*joystick_y + turn_out;
           double right_out = joystick_y*joystick_y*joystick_y - turn_out;

        m_leftdrive -> Set(left_out);
        m_leftdrive2 -> Set(left_out);
        m_rightdrive -> Set(right_out);
        m_rightdrive2 -> Set(right_out);

    }

    void Drive::gyro_reset(){
        s_gyro -> Reset();

    }

    /* Path Following */ 
           
            void Drive::drive_PID(vector<double>value_in, int count) {
            
            //Breakdown input vector
            double setpoint_left_pos = value_in[0];
            double setpoint_right_pos = value_in[1];
            double setpoint_left_speed = value_in[2];
            double setpoint_right_speed = value_in[3];
            double heading = value_in[4];

            if(count ==0){
                s_gyro->Reset();
                s_leftdrive_enc -> SetPosition(0);
                s_rightdrive_enc -> SetPosition(0);
            }

            double encoder_val_left = s_leftdrive_enc -> GetPosition();
            double encoder_val_right = s_rightdrive_enc -> GetPosition();
            double encoder_speed_left = s_leftdrive_enc -> GetVelocity();
            double encoder_speed_right = s_rightdrive_enc -> GetVelocity();
            double gyro_val = s_gyro->GetAngle();

            double error_left_pos = setpoint_left_pos - encoder_val_left;
            double error_right_pos = setpoint_right_pos - encoder_val_right;
            double error_left_speed = setpoint_left_speed - encoder_speed_left;
            double error_right_speed = setpoint_right_speed - encoder_speed_right;
            double error_heading = heading - gyro_val;
            //frc::SmartDashboard::PutNumber("error left", error_left_pos); 
            //frc::SmartDashboard::PutNumber("error right", error_right_pos);
            //frc::SmartDashboard::PutNumber("error left speed", error_left_speed);
            //frc::SmartDashboard::PutNumber("error right speed", error_right_speed); 
            //frc::SmartDashboard::PutNumber("error heading", error_heading);
            //frc::SmartDashboard::PutNumber("left speed", encoder_speed_left);
            //frc::SmartDashboard::PutNumber("right speed", encoder_speed_right);

            double max_speed = frc::SmartDashboard::GetNumber("KVelo In", 3209.6);
            double kp_pos = frc::SmartDashboard::GetNumber("KPos_in", 0); 
            double kp_speed = -1/(max_speed);
            double kph = frc::SmartDashboard::GetNumber("KPH_in", 0); 

            double output_left = (error_left_pos * kp_pos) + kp_speed*setpoint_left_speed;
            double output_right = (error_right_pos * kp_pos) + kp_speed*setpoint_right_speed;
            output_left = Remap_Val( output_left,.9);
            output_right = Remap_Val( output_right,.9);

            double turn_val = kph * error_heading;

            turn_val = Remap_Val (turn_val, 0.75);
        //frc::SmartDashboard::PutNumber("Output left", output_left + turn_val);
        //frc::SmartDashboard::PutNumber("Output right", output_right - turn_val);
        m_leftdrive->Set(output_left + turn_val);
        m_leftdrive2->Set(output_left + turn_val);
        m_rightdrive->Set(output_right - turn_val);
        m_rightdrive2->Set(output_right - turn_val);

}
  

    void Drive::dashboard(){
        frc::SmartDashboard::PutNumber("Gryo",s_gyro -> GetAngle());
        frc::SmartDashboard::PutNumber("Left Climb Enc", s_leftclimber_enc -> GetPosition());
        frc::SmartDashboard::PutNumber("Right Climb Enc", s_rightclimber_enc -> GetPosition());
        frc::SmartDashboard::PutNumber("Right Drive Enc", s_rightdrive_enc -> GetPosition()); 
        frc::SmartDashboard::PutNumber("Left Drive Enc", s_leftdrive_enc -> GetPosition());       
    }

    int Drive::climber_extend_nolimit(){

            int output;
            
            p_climberlock-> Set(frc::DoubleSolenoid::Value::kReverse);
          
    
            if (climb_lock > 2){
                m_leftclimb -> Set(1);
                m_rightclimb -> Set(-1);
            }
            else{
                m_leftclimb -> Set(0);
                m_rightclimb -> Set(0);
            }
            
            climb_lock ++;
        

            return output;

            }

        

        int Drive::climber_retract_nolimit(){
           
            //unlock climbers and retract
            int output;
        
            output = 0;
            p_climberlock-> Set(frc::DoubleSolenoid::Value::kReverse);

            if (climb_lock > 2){
                m_leftclimb -> Set(-1);
                m_rightclimb -> Set(1);
            }
            else{
                m_leftclimb -> Set(0);
                m_rightclimb -> Set(0);
            }
            climb_lock ++;
            
           
            return output;
            
        }

    bool Drive::turnto_gyro (double angle){
        double angle_in = s_gyro -> GetAngle();
        bool atangle = false;

        double error = angle - angle_in;
        double kp = 0.001;
        kp = frc::SmartDashboard::GetNumber("KP_Gyro", -0.0027); 
  

        double out = kp * error;

            if (abs(out)<0.1){
                out = 0;
                atangle = true;
            }

        m_leftdrive -> Set(out);
        m_rightdrive -> Set(-out);
        m_leftdrive2 -> Set(out);
        m_rightdrive2 -> Set(-out);

        return atangle;
    }

      bool Drive::driveto_distance (double distance){
        double distance_in = s_leftdrive_enc -> GetPosition();
        bool atdistance = false;
        distance_in = -1*distance_in;

        distance = 0.183*distance;
        double error = distance - distance_in;
        double kp = 0.001;
        kp = frc::SmartDashboard::GetNumber("KP_DriveDist", -0.07); 
        frc::SmartDashboard::PutNumber("Drive Distance In",distance);
        frc::SmartDashboard::PutNumber("Drive Distance",distance_in);
        double out = kp * error;

        frc::SmartDashboard::PutNumber("Drive Out1",out);

            if (abs(out)<0.1){
                out = 0;
                atdistance = true;
            }

        out = Remap_Val(out,0.99);
        drive_straight(false,out);

        return atdistance;
    }
void Drive::reset_drive_s(){
    s_gyro->Reset();
    s_leftdrive_enc -> SetPosition(0);
    s_rightdrive_enc -> SetPosition(0);
}
