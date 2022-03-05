// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* ---------------------------------------------------------------------------
------------------------------TEAM 573 NAMING CONVENTION---------------------------------
MOTORS: m_function  ex: m_leftdrive
SENSORS: s_type  ex: s_gyro
CONTROLLERS: c#_type_name  ex: c1_btn_a
PNEUMATICS: p_function ex: p_intake
BRANCHES OF CODE: section/what you're working on     ex: Drive/JoystickControl
also: merge into main before each event, Before event create event branch and merge every night
--------------------------------------------------------------------------------
*/

#include "Robot.h"
#include "Drive.h"
#include "Appendage.h"
#include "Led.h"
#include <fmt/core.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <tuple>

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  m_chooser.AddOption(kAutoNameCustom1, kAutoNameCustom1);
  m_chooser.AddOption(kAutoNameCustom2, kAutoNameCustom2); //Path test

  m_alliance.SetDefaultOption(kBlue, kBlue);
  m_alliance.AddOption(kRed, kRed);

  frc::SmartDashboard::PutNumber("Auto delay", auto_timer);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutData("Alliance Color", &m_alliance);
  alliance_color = "red";  // Default evaluated in auto and teleop inits
  turret_direction = true; // Initial turrent scan direction
  shooter_trim = 0;
  frc::SmartDashboard::PutNumber("Shooter Trim", shooter_trim);

// Initial pnematic states
  MyAppendage.Intake_Up();
  MyDrive.climber_hold();
  MyDrive.climber_tiltin();
  MyAppendage.Articulate(4);

  // Dashboard input creations
  MyAppendage.DashboardCreate();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{

  auto_ball_pickedup = false;
  firsttimethru = true;
  counter = 0;

  // Get alliance station color

  static auto color = frc::DriverStation::GetAlliance();
   m_allianceselected = m_alliance.GetSelected();
  if (m_allianceselected == "Blue")
  {
    alliance_color = "blue";
  }

  else{
    alliance_color = "red";
  }



  shooter_trim = frc::SmartDashboard::GetNumber("Shooter Trim", 0);
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);

  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic(){

  //Compressor Code
  compressor.EnableAnalog(units::pounds_per_square_inch_t(85), units::pounds_per_square_inch_t (120));

  //Reset shooter variables
  bool align = false;
  bool atspeed = false;


  // -------- Read in Shooter camera Stuff -----------------------------------------------

  std::shared_ptr<nt::NetworkTable> table_s = nt::NetworkTableInstance::GetDefault().GetTable("limelight-shooter");
  table_s->PutNumber("ledMode", 0);
  table_s->PutNumber("camMode", 0);

  // -----------PIPELINE STUFF-----------//
      
      table_s -> PutNumber("pipeline", 0);

  //--------CAMERA VALUES-----------------//
  float shooter_camera_x = table_s->GetNumber("tx", 0);

  float shooter_camera_exist = table_s->GetNumber("tv", 0);
  // float image_size = table->GetNumber("ta", 0);
  float shooter_camera_y = table_s->GetNumber("ty", 0);

  double distance = MyAppendage.Get_Distance(shooter_camera_y);

  // ----------------------------------------------------------

  // -------- Read in Intake camera Stuff -----------------------------------------------

  std::shared_ptr<nt::NetworkTable> table_i = nt::NetworkTableInstance::GetDefault().GetTable("limelight-intake");
  table_i->PutNumber("ledMode", 0);
  table_i->PutNumber("camMode", 0);

  // -----------PIPELINE STUFF-----------//

   if (alliance_color == "red"){
        table_i -> PutNumber("pipeline", 0);
    }
      else {
        table_i -> PutNumber("pipeline", 1);
      }


  //--------CAMERA VALUES-----------------//
  float intake_camera_x = table_i->GetNumber("tx", 0);
  float intake_camera_exist = table_i->GetNumber("tv", 0);
  // float image_size = table->GetNumber("ta", 0);
  //float intake_camera_y = table_i->GetNumber("ty", 0);
  auto_timer = frc::SmartDashboard::GetNumber("Auto delay", 0)*50;

  // ----------------------------------------------------------
  if (counter >= auto_timer) {

    if (m_autoSelected == kAutoNameCustom){
      // 2 Ball Autonomous

      if (counter - auto_timer < 3){
        MyAppendage.Intake_Down();
        MyAppendage.Intake_In();
      }

      else if (intake_camera_exist == 1 && !auto_ball_pickedup){
        MyDrive.camera_intake(intake_camera_x, -0.7);
        MyAppendage.Intake_Down();
        bool LightGate_val = MyAppendage.Intake_In();

        /* Lightgate not working yet.
        if (LightGate_val){
          MyAppendage.Intake2_In();
        }

          else{
            MyAppendage.Intake2_Off();
          }
        */
      }
      else{
        auto_ball_pickedup = true;
        MyAppendage.Intake_Off();
        MyAppendage.Intake_In();
        double distance = MyAppendage.Get_Distance(shooter_camera_y);
        tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false);
        MyAppendage.Articulate(distance);
        bool atspeed = MyAppendage.Shooter_Encoder_distance(distance,shooter_trim);
        MyDrive.Joystick_Drive(0,0);

        if (align && atspeed){
          MyAppendage.Feeder_In();
          MyAppendage.Intake2_In();
        }
        else{
          MyAppendage.Feeder_Off();
          MyAppendage.Intake2_Off();
        }
      }
    }
    else if (m_autoSelected == kAutoNameCustom1){
      // 4  Ball Autonomous this cannot be put on delay we need the whole time

      int FirstSectionOffset = 50*6; // Gives 6sec for first part of auto to take place
      int SecondSelectionOffset = 0;

      if (counter < FirstSectionOffset){
        // 50 = 1 second

        if (counter < 3){
        MyAppendage.Intake_Down();
        MyAppendage.Intake_In();
        }

        else if (intake_camera_exist == 1 && !auto_ball_pickedup){

          MyDrive.camera_intake(intake_camera_x, -0.7);
          MyAppendage.Intake_Down();
          bool LightGate_val = MyAppendage.Intake_In();
          /*
          if (LightGate_val){ //Light gate not working right now
            MyAppendage.Intake2_In();
          }

            else{
              MyAppendage.Intake2_Off();
            }
            */
        }
        else{
          auto_ball_pickedup = true;
          MyAppendage.Intake_Off();
          MyAppendage.Intake_In();
          double distance = MyAppendage.Get_Distance(shooter_camera_y);
          tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false);
          MyAppendage.Articulate(distance);
          bool atspeed = MyAppendage.Shooter_Encoder_distance(distance,shooter_trim);
          MyDrive.Joystick_Drive(0,0);

          if (align && atspeed){
            MyAppendage.Feeder_In();
            MyAppendage.Intake2_In();
          }
          else{
            MyAppendage.Feeder_Off();
            MyAppendage.Intake2_Off();
          }
        }

      }
      // Second Half of 4 ball auto
    else {
      auto_ball_pickedup = false;
      vector <double> Length = MyPath.ReturnTableVal(counter - FirstSectionOffset, 1, true);
      int length = round (Length [0]);

      if (counter - FirstSectionOffset < length){
        vector <double> Table_Values = MyPath.ReturnTableVal(counter - FirstSectionOffset, 1, false);
        MyDrive.drive_PID(Table_Values, counter - FirstSectionOffset);
      }

      else {
        if (intake_camera_exist == 1 & !auto_ball_pickedup){
          
          MyDrive.camera_intake(intake_camera_x, -0.7);
          MyAppendage.Intake_Down();
          bool LightGate_val = MyAppendage.Intake_In();

          /*if (LightGate_val){ // Light gate not working
            MyAppendage.Intake2_In();
          }
            else{
              MyAppendage.Intake2_Off();
            }*/
        }

        else{
            auto_ball_pickedup = true;
            if(firsttimethru){
              firsttimethru = false;
              SecondSelectionOffset = counter;
            }
            vector <double> Length2 = MyPath.ReturnTableVal(counter - SecondSelectionOffset, 2, true);
            int length2 = round (Length2 [0]);

            if (counter - SecondSelectionOffset < length2){
              vector <double> Table_Values = MyPath.ReturnTableVal(counter - SecondSelectionOffset, 2, false);
              MyDrive.drive_PID(Table_Values, counter - SecondSelectionOffset);
            }

            else{
              MyAppendage.Intake_Off();
              MyAppendage.Intake_In();
              double distance = MyAppendage.Get_Distance(shooter_camera_y);
              tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false);
              MyAppendage.Articulate(distance);
              bool atspeed = MyAppendage.Shooter_Encoder_distance(distance,shooter_trim);
              MyDrive.Joystick_Drive(0,0);

              if (align && atspeed){
                MyAppendage.Feeder_In();
                MyAppendage.Intake2_In();
              }
              else{
                MyAppendage.Feeder_Off();
                MyAppendage.Intake2_Off();
              }
            }
          }

        }
    } // End 2nd half of 4ball auto
  } // End 4 ball auto
///////////////////////////////////////////////////////////////////////////////
  else if(m_autoSelected == kAutoNameCustom2){

   vector <double> Length = MyPath.ReturnTableVal(counter, 1, true);
      int length = round (Length [0]);

    if (counter < length){
      vector <double> Table_Values = MyPath.ReturnTableVal(counter, 1, false);
      MyDrive.drive_PID(Table_Values, counter);
    }

    else{
      MyDrive.Joystick_Drive(0,0);
    }
  }
  else{ // Simple drive straight auto

      if (counter < (100 + auto_timer))
      { // 100 = 2 seconds
        MyDrive.Joystick_Drive(-0.5, -0.5);
      }
      else
      {
        MyDrive.Joystick_Drive(0, 0);
      }
    }
  }

  counter++;
} // End of Auto Periodic

void Robot::TeleopInit()
{

  // Setting teleop variables
  climber_state = 0;
  climber_count = 0;
  drive_straight_first = true;
  endgame_unlock = false;
  shooter_test = false;

  shooter_trim = frc::SmartDashboard::GetNumber("Shooter Trim", 0);

  // Get alliance station color
  static auto color = frc::DriverStation::GetAlliance();
  m_allianceselected = m_alliance.GetSelected();
  if (m_allianceselected == "Blue")
  {
    alliance_color = "blue";
    //sfrc::SmartDashboard::PutString("Alliance","Blue");
  }
  else{
    alliance_color = "red";
    //frc::SmartDashboard::PutString("Alliance","Red");
  }
  frc::SmartDashboard::PutString("Alliance",alliance_color);
}
void Robot::TeleopPeriodic(){


  //Compressor Code
  compressor.EnableAnalog(units::pounds_per_square_inch_t(85), units::pounds_per_square_inch_t (120));

  //Reset shooter variables

  bool align = false;
  bool atspeed = false;

  //********** Read in Joystick Values ******************************************
  //------------- Driver Controller ---------------------------------------------

  double c1_joy_leftdrive = controller1.GetRawAxis(1);
  double c1_joy_rightdrive = controller1.GetRawAxis(5);
  bool c1_btn_back = controller1.GetRawButton(7);
  bool c1_btn_start = controller1.GetRawButton(8);
  double c1_righttrigger = controller1.GetRawAxis(3);
  double c1_lefttrigger = controller1.GetRawAxis(2);
  bool c1_leftbmp = controller1.GetRawButton(5);
  bool c1_rightbmp = controller1.GetRawButton(6);
  bool c1_btn_b = controller1.GetRawButton(2);
  bool c1_btn_x = controller1.GetRawButton(3);
  bool c1_btn_a = controller1.GetRawButton(1);
  bool c1_btn_y = controller1.GetRawButton(4);

  //-----------------------------------------------------------------------------
  //------------ Operator Controller --------------------------------------------
  // double c2_joy_left = controller2.GetRawAxis(1);
  bool c2_btn_a = controller2.GetRawButton(1);
  bool c2_btn_b = controller2.GetRawButton(2);
  bool c2_btn_y = controller2.GetRawButton(4);
  bool c2_btn_x = controller2.GetRawButton(3);
  // bool c2_btn_lb = controller2.GetRawButton(5);
  // bool c2_btn_rb = controller2.GetRawButton(6);
  double c2_dpad = controller2.GetPOV(0);
  bool c2_btn_back = controller2.GetRawButton(7);
  bool c2_btn_start = controller2.GetRawButton(8);

  bool c2_rightbumper = controller2.GetRawButton(6);
  bool c2_leftbumper = controller2.GetRawButton(5);

  double c2_right_trigger = controller2.GetRawAxis(3);
  double c2_left_trigger = controller2.GetRawAxis(2);
  //----------------------------------------------------------------------------

  // -------- Read in Shooter camera Stuff -----------------------------------------------

  std::shared_ptr<nt::NetworkTable> table_s = nt::NetworkTableInstance::GetDefault().GetTable("limelight-shooter");
  //IP Address: 10.5.73.11
  table_s->PutNumber("ledMode", 0);
  table_s->PutNumber("camMode", 0);

  // -----------PIPELINE STUFF-----------//
  table_s->PutNumber("pipeline", 0);

  //--------CAMERA VALUES-----------------//
  float shooter_camera_x = table_s->GetNumber("tx", 0);
  float shooter_camera_exist = table_s->GetNumber("tv", 2); // If value 2 means no camera data following
  // float image_size = table->GetNumber("ta", 0);
  float shooter_camera_y = table_s->GetNumber("ty", 0);
  double distance = MyAppendage.Get_Distance(shooter_camera_y);

  // ----------------------------------------------------------

  // -------- Read in Intake camera Stuff -----------------------------------------------

  std::shared_ptr<nt::NetworkTable> table_i = nt::NetworkTableInstance::GetDefault().GetTable("limelight-intake");
  //IP Address: 10.5.73.12
  table_i -> PutNumber("ledMode", 0);
  table_i -> PutNumber("camMode", 0);

  // -----------PIPELINE STUFF-----------//


  if (alliance_color == "red"){
    table_i -> PutNumber("pipeline", 0);
  }
  else {
    table_i -> PutNumber("pipeline", 1);
  }

  //--------CAMERA VALUES-----------------//
  float intake_camera_x = table_i -> GetNumber("tx", 0);

  float intake_camera_exist = table_i -> GetNumber("tv", 2); // If value 2 means no camera data following
  // float image_size = table->GetNumber("ta", 0);
  //float intake_camera_y = table_i -> GetNumber("ty", 0);


  // ----------------------------------------------------------

  /*--------------------- DRIVE CODE -------------------------- */

  if (c1_btn_b) // Drive Straight with Gyro
  {
    MyDrive.drive_straight(drive_straight_first, c1_joy_leftdrive);

    drive_straight_first = false;
  }

  else if (c1_btn_a) // Auto pickup with camera
  {
    if (intake_camera_exist == 1){
    MyDrive.camera_intake(intake_camera_x, c1_joy_leftdrive);
    }  

    else {
      MyDrive.Joystick_Drive(c1_joy_leftdrive, c1_joy_leftdrive);
    }
  }

  else // Joystick drive
  {
    drive_straight_first = true;

    MyDrive.Joystick_Drive(c1_joy_leftdrive, c1_joy_rightdrive);
  }
  /* ---------------------- CLIMBER CODE -----------------------------*/

  // Climber lock / unlock check
  if (c1_btn_back && c1_btn_start)
  {
    endgame_unlock = true;
  }
  if (endgame_unlock && c1_btn_y){
    endgame_unlock = false;
  }

  if (endgame_unlock){
    bool output;
    bool output_1;
    
    // Extend / Retract Arms
    if (c1_righttrigger > 0.5)
    {
      MyDrive.climber_extend();
    }

    else if (c1_lefttrigger > 0.5)
    {
      MyDrive.climber_retract();
    }

    else if (c1_btn_x){ //Auto climb

      switch (climber_state){
        case 0:
          tie(output,output_1) = MyDrive.climber_setpoint("retract");
            
            climber_count = 0;

            if (output){
              climber_state ++;
            }
            break;

            case 1:
          tie(output,output_1) = MyDrive.climber_setpoint("extend");
            
            if (output_1){
              climber_state ++;
            }
            break;

            case 2:
          tie(output,output_1) = MyDrive.climber_setpoint("extend");
            
            MyDrive.climber_tiltout();

            if (output){
              climber_state ++;
            }
            break;

            case 3:
          tie(output,output_1) = MyDrive.climber_setpoint("extend");
            MyDrive.climber_tiltin();

            if (output && climber_count > 5){
              climber_state = 0;
            }
              climber_count ++;
              break;
      }
    }
    else{
      MyDrive.climber_hold();
    }

    // Tilt Climber Arms
    if (c1_leftbmp){
      MyDrive.climber_tiltin();
    }

    else if (c1_rightbmp){
      MyDrive.climber_tiltout();
    }

  }
// -------------------------------------------------------------------

//--------------------Intake Code -----------------------------------
// Extend / Retract Intake
//Run intake
if (c2_leftbumper){
    MyAppendage.Intake_Down();
    MyAppendage.Intake_In();
}
else if(c2_btn_y){
  MyAppendage.Intake_Down();
  MyAppendage.Intake_Out();
}
else{
  MyAppendage.Intake_Up();
  MyAppendage.Intake_Off();
}
/*
// Run Intake In / Out
if (c2_rightbumper){
  bool LightGate_val = MyAppendage.Intake_In();

  
  if (LightGate_val && !shooter_test){
    MyAppendage.Intake2_In();
  }
  else{
    MyAppendage.Intake2_Off();
    frc::SmartDashboard::PutString("Intake State", "Off");
  }
}
else if (c2_btn_y){
  MyAppendage.Intake_Out();
  
}
else{
  MyAppendage.Intake_Off();
}*/

  //--------------------Shooter Code -----------------------------------

// Shooter Trim 

if (c2_dpad > 80 && c2_dpad < 100){ // Right on dpad
  shooter_trim ++;
}
else if(c2_dpad > 260 && c2_dpad < 280){ // Left on dpad
  shooter_trim --;
}

frc::SmartDashboard::PutNumber("Shooter Trim", shooter_trim);

// Get into and out of shooter test mode
if (c2_btn_start && c2_btn_back){
  shooter_test = true;
}

if (c2_btn_x && shooter_test){
  shooter_test = false;

}

// Shooter state code blocks 
if (endgame_unlock){ // Endgame shooter
  //MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, true);
  MyAppendage.Rotate_Off(); // Only for testing, line above should be used for competition.
  MyAppendage.Shooter_Off();
  MyAppendage.Feeder_Off();
  MyAppendage.Intake2_Off();
}

else if (shooter_test){ // Shooter Test

  // Turret Test Section
  if (c2_btn_start){
    MyAppendage.Rotate_left();
  }
  else if (c2_btn_back){
    MyAppendage.Rotate_right();
  }
  else{
    MyAppendage.Rotate_Off();
  }

// Shooter wheel test section
  if (c2_left_trigger > 0.5){
    MyAppendage.Shooter_Encoder();
  }
  else{
    MyAppendage.Shooter_Off();
  }

// Tower wheels test section
  if (c2_right_trigger > 0.5){
    MyAppendage.Feeder_In();
    MyAppendage.Intake2_In();
  }
  else{
    MyAppendage.Feeder_Off(); 
    MyAppendage.Intake2_Off();
  }

// Hood Pnematic Test section
  if(c2_btn_a){
    MyAppendage.Articulate(12); // 12 is just some random number needs to get updated.
  }
  if(c2_btn_b){
    MyAppendage.Articulate(150); // 150 is just some random number needs to get updated.
  }
}

else if (c2_btn_a){
  //Low Fixed shoot

  tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, true, false);

  atspeed = MyAppendage.Shooter_Encoder_distance(24, 0);
  MyAppendage.Articulate(12); //harcode for close shot

  if(align && atspeed && (c2_right_trigger > 0.5)){ // Shoot ball
    MyAppendage.Feeder_In();
    MyAppendage.Intake2_In();
  }
  else{
    MyAppendage.Feeder_Off();
    MyAppendage.Intake2_Off();
  }

}

else if (c2_btn_x){
  //shoot out

  atspeed = MyAppendage.Shooter_Encoder_distance(24, 0);

  if( (c2_right_trigger > 0.5)){ // Shoot ball
    MyAppendage.Feeder_In();
    MyAppendage.Intake2_In();
  }
  else{
    MyAppendage.Feeder_Off();
    MyAppendage.Intake2_Off();
  }

}

else if (c2_btn_b){

// Test turret camera tracking

  tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false);
  

  /*//High Fixed shoot

  tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, true, false);

  atspeed = MyAppendage.Shooter_Encoder();
  MyAppendage.Articulate(144); //harcode for far shot

  if(align && atspeed && (c2_right_trigger > 0.5)){ // Shoot ball
    MyAppendage.Feeder_In();
  }
  else{
      MyAppendage.Feeder_Off();
  }*/

}


else {
  
  if (c2_left_trigger >= 0.5)
  {
    //Get shooter aligned and up to speed
    tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false);
    atspeed = MyAppendage.Shooter_Encoder_distance(distance,shooter_trim);
    MyAppendage.Articulate(distance);

    if(align && atspeed && (c2_right_trigger > 0.5)){ // Shoot ball
      MyAppendage.Feeder_In();
      MyAppendage.Intake2_In();
    }
    else{
      MyAppendage.Feeder_Off();
      MyAppendage.Intake2_Off();
    }
  }
  else {
    MyAppendage.Shooter_Off();
    MyAppendage.Rotate_Off();
    MyAppendage.Feeder_Off();
    MyAppendage.Intake2_Off();

  }
}
// -------------------------------------------------------------------

//---------------------LED CODE----------------------------------

if (endgame_unlock){
  MyLed.led_control("Rainbow");
}

else if (align && !atspeed){
  MyLed.led_control("Yellow");
}

else if (!align && atspeed){
  MyLed.led_control("Red");
}

else if (align && atspeed){
  MyLed.led_control("Green");
}

else if (intake_camera_exist == 1){
  MyLed.led_control("White");
}

else{
  MyLed.led_control("Black");
}

// -------------------------------------------------------------------

// --------- dashboard code ---------------

frc::SmartDashboard::PutBoolean("Endgame State", endgame_unlock);
frc::SmartDashboard::PutBoolean("Shooter Test State", shooter_test);
frc::SmartDashboard::PutBoolean("Shooter At Speed", atspeed);
frc::SmartDashboard::PutBoolean("Shooter Aligned", align);
frc::SmartDashboard::PutNumber("Camera Distance", distance);

//Drive Current Compares
/*
MyLog.CurrentCompare(19, 7);
MyLog.CurrentCompare(18, 8);
MyLog.CurrentCompare(0, 9);
MyLog.CurrentCompare(1, 10);

//Shooter Current Compares

MyLog.CurrentCompare(13, 14);
MyLog.CurrentCompare(14, 2);
*/

//MyLog.Dashboard();
//MyLog.PDPTotal();
MyDrive.dashboard();
MyAppendage.dashboard();


// ------------------------------------------
} // end of teleop periodic

void Robot::DisabledInit() {
  MyDrive.climber_hold();
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
