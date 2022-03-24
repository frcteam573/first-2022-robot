#ifndef Appendage_H
#define Appendage_H

#pragma once
#include "rev/CANSparkMax.h"
#include "rev/SparkMaxRelativeEncoder.h"
#include <frc/DoubleSolenoid.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <tuple>
#include <frc/DigitalInput.h>
#include <frc/util/color.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

using namespace std;

class Appendage
{
private:
    // Define motor, sensors, and pnematic pointers here
    frc::DoubleSolenoid *p_Intake;
    frc::DoubleSolenoid *p_Hood;

    rev::CANSparkMax *m_Intake1;
    rev::CANSparkMax *m_Intake2;
    rev::CANSparkMax *m_Shooter1;
    rev::CANSparkMax *m_Shooter2;
    rev::CANSparkMax *m_Feeder;
    rev::CANSparkMax *m_Susan;

    rev::ColorSensorV3 *m_colorSensor;
    rev::ColorMatch *m_colorMatcher;


    rev::SparkMaxRelativeEncoder *s_Shooter_Encoder;
    rev::SparkMaxRelativeEncoder *s_Susan_Encoder;
    rev::SparkMaxRelativeEncoder *s_Hood_Encoder;

    frc::DigitalInput *s_LightGate;
   

public:
    Appendage();
    bool Intake_In(char color_in = 'W');
    void Intake_Out();
    void Intake_Off();

    void Intake2_In();
    void Intake2_Out();
    void Intake2_Off();    

    void Feeder_In();
    void Feeder_Out();
    void Feeder_Off();

    void Intake_Up();
    void Intake_Down();
    double Remap_Val(double i, double threshold);
    bool Shooter_Encoder_distance (double distance, double trim);

    bool Shooter_Encoder();
    void Shooter_Off();
    bool color_in();

    void controlpanel_colorsense_init();
    char controlpanel_colorsense_periodic();

    double Get_Distance(double camera_y);
    std::tuple<bool, bool> Rotate(double camera_exists, double camera_x, bool direction, bool lowgoal, bool endgame, bool kAuto);
    void Rotate_Off();
    void Articulate(double distance);

    void Rotate_left();
    void Rotate_right();

    void DashboardCreate();

    static double shooter_p_in;
    static double shooter_target_in;
    static double shooter_f_in;
    static double k_turret_cam;
    static double k_turret_enc;
    static double turret_max_enc;
    static double turret_min_enc;
    static double turret_enc_deadzone;
    static double turret_cam_deadzone;
    static double feedfor;
    int ct;
    double shooterout_old;

    void dashboard();

    //Any updates here also have to be done in controlpanel_colorsense_init
    static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
    static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
    static constexpr frc::Color kWhiteTarget = frc::Color(0.365, 0.464, 0.169);



};
#endif


