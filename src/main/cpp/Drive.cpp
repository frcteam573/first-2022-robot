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
        m_leftdrive -> SetInverted(true);
        m_leftdrive2 -> SetInverted(true);
        m_rightdrive = new rev::CANSparkMax{rightdriveID, rev::CANSparkMax::MotorType::kBrushless};
        m_rightdrive2 = new rev::CANSparkMax{rightdriveID2, rev::CANSparkMax::MotorType::kBrushless};

        s_gyro = new frc::ADXRS450_Gyro(frc::SPI::Port::kOnboardCS0);

    // Climb motors, sensors, and pneumatics
    
        m_leftclimb = new rev::CANSparkMax{leftclimbID, rev::CANSparkMax::MotorType::kBrushless};
        m_rightclimb = new rev::CANSparkMax{rightclimbID, rev::CANSparkMax::MotorType::kBrushless};
        m_leftclimb -> SetInverted(true);  

        p_climberlock = new frc::DoubleSolenoid{frc::PneumaticsModuleType::REVPH, climberlockIDa, climberlockIDb};  
        p_climbertilt = new frc::DoubleSolenoid{frc::PneumaticsModuleType::REVPH, climber_tiltIDa, climber_tiltIDb};

        s_leftclimber_enc = new rev::SparkMaxRelativeEncoder{m_leftclimb -> GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,42)};
        s_rightclimber_enc = new rev::SparkMaxRelativeEncoder{m_rightclimb -> GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,42)};

        s_leftdrive_enc = new rev::SparkMaxRelativeEncoder{m_leftdrive -> GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,42)};
        s_rightdrive_enc = new rev::SparkMaxRelativeEncoder{m_rightdrive -> GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,42)};

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

    m_leftdrive -> Set(left_out);
    m_leftdrive2 -> Set(left_out);
    m_rightdrive -> Set(right_out);
    m_rightdrive2 -> Set(right_out);
}

//DRIVE STRAIGHT//
        void Drive::drive_straight(bool first, double joystick_y){
       
            if (first)
            {
                s_gyro -> Reset();
            }
            
        double error = 0-s_gyro -> GetAngle();
        double constant = 0.3;

        double turn_out = constant*error; 

           double left_out = joystick_y + turn_out;
           double right_out = joystick_y + turn_out;

        m_leftdrive -> Set(left_out);
        m_leftdrive2 -> Set(left_out);
        m_rightdrive -> Set(right_out);
        m_rightdrive2 -> Set(right_out);

        }

/* CLIMBING */

        void Drive::climber_extend(){

            if (s_leftclimber_enc->GetPosition() < 4000) {
                //unlock climbers and extend
                p_climberlock-> Set(frc::DoubleSolenoid::Value::kReverse);
                
                m_leftclimb -> Set(1);
                m_rightclimb -> Set(1);

            }

                else{
                    p_climberlock-> Set(frc::DoubleSolenoid::Value::kForward);
                    
                    m_leftclimb -> Set(0);
                    m_rightclimb -> Set(0);

                }
        }

        void Drive::climber_retract(){
           
            //unlock climbers and retract
            p_climberlock-> Set(frc::DoubleSolenoid::Value::kReverse);
            
            m_leftclimb -> Set(-1);
            m_rightclimb -> Set(-1);
        }

        void Drive::climber_hold(){
           
            //unlock climbers and hold
            p_climberlock -> Set(frc::DoubleSolenoid::Value::kForward);
            
                m_leftclimb -> Set(0);
                m_rightclimb -> Set(0);
        }

        void Drive::climber_tiltin(){  
            p_climbertilt -> Set(frc::DoubleSolenoid::Value::kForward);
        }

        void Drive::climber_tiltout(){  
            p_climbertilt -> Set(frc::DoubleSolenoid::Value::kReverse);
        }

/* CAMERA INTAKE */

    void Drive::camera_intake(double camera_x, double joystick_y){
       
       double error = 0-camera_x;
       double constant = 0.3;

        double turn_out = constant*error; 

           double left_out = joystick_y + turn_out;
           double right_out = joystick_y + turn_out;

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
                //Gyro->Reset();
                s_leftdrive_enc -> SetPosition(0);
                s_rightdrive_enc -> SetPosition(0);
            }

            double encoder_val_left = s_leftdrive_enc -> GetPosition();
            double encoder_val_right = s_rightdrive_enc -> GetPosition();
            //double encoder_speed_left = s_leftdrive_enc -> GetRate();
            //double encoder_speed_right = s_rightdrive_enc -> GetRate();
            double gyro_val = s_gyro->GetAngle();

            double error_left_pos = setpoint_left_pos - encoder_val_left;
            double error_right_pos = setpoint_right_pos - encoder_val_right;
            //double error_left_speed = setpoint_left_speed - encoder_speed_left;
            //double error_right_speed = setpoint_right_speed - encoder_speed_right;
            double error_heading = heading - gyro_val;
            
            
            double max_speed = frc::SmartDashboard::GetNumber("p input 2", 8250);//9000,8000//frc::SmartDashboard::GetNumber("p input 2", 9750);//8250
            double kp_speed = -1/(max_speed);
            double kp_pos = 0; //-0.002;//frc::SmartDashboard::GetNumber("p input", -0.025);//-0.074;
            
            double kph = frc::SmartDashboard::GetNumber("p input", -0.0072);//-0.0072;//-0.01;  //0.01;

            double output_left = (error_left_pos * kp_pos) + kp_speed*setpoint_left_speed;
            double output_right = (error_right_pos * kp_pos) + kp_speed*setpoint_right_speed;

            double turn_val = kph * error_heading;

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