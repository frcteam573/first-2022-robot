#include "Drive.h"
#include "rev/CANSparkMax.h"

using namespace std;

Drive::Drive(){
    int leftdriveID = 9;
    int leftdriveID2 = 10;
    int rightdriveID = 7;
    int rightdriveID2 = 8;

    // Define motors, sensors, and pneumatics here
    m_leftdrive = new rev::CANSparkMax{leftdriveID, rev::CANSparkMax::MotorType::kBrushless}; //actually 1
    m_leftdrive2 = new rev::CANSparkMax{leftdriveID2, rev::CANSparkMax::MotorType::kBrushless};
    m_leftdrive->SetInverted(true);
    m_leftdrive2->SetInverted(true);
    m_rightdrive = new rev::CANSparkMax{rightdriveID, rev::CANSparkMax::MotorType::kBrushless};
    m_rightdrive2 = new rev::CANSparkMax{rightdriveID2, rev::CANSparkMax::MotorType::kBrushless};
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
