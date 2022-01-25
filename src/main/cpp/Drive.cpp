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
    int rightdriveencID_a = 7;
    int rightdriveencID_b = 8;
    int leftdriveencID_a = 9;
    int leftdriveencID_b = 10;


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
           
            //unlock climbers and extend
            p_climberlock-> Set(frc::DoubleSolenoid::Value::kReverse);
            
            m_leftclimb -> Set(1);
            m_rightclimb -> Set(1);
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

    void Drive::dashboard(){
        frc::SmartDashboard::PutNumber("Gryo",s_gyro -> GetAngle());
        frc::SmartDashboard::PutNumber("Left Climb Enc", s_leftclimber_enc -> GetPosition());
        frc::SmartDashboard::PutNumber("Right Climb Enc", s_rightclimber_enc -> GetPosition());
        frc::SmartDashboard::PutNumber("Right Drive Enc", s_rightdrive_enc -> GetPosition()); 
        frc::SmartDashboard::PutNumber("Left Drive Enc", s_leftdrive_enc -> GetPosition());       
    }