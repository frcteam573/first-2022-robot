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
#include <frc/livewindow/LiveWindow.h>
#include <tuple>

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);// Straight
  m_chooser.AddOption(kAutoName2Ball, kAutoName2Ball); // 2 ball
  m_chooser.AddOption(kAutoName3BallPath, kAutoName3BallPath); // 3 ball
  m_chooser.AddOption(kAutoName4BallNoPath, kAutoName4BallNoPath); //4 Ball 2
  m_chooser.AddOption(kAutoNameCustom2, kAutoNameCustom2); 

  
  frc::LiveWindow::DisableAllTelemetry();
  //frc::SmartDashboard::PutBoolean("St Test", false);
  m_alliance.SetDefaultOption(kBlue, kBlue);
  m_alliance.AddOption(kRed, kRed);

  frc::SmartDashboard::PutNumber("Auto delay", auto_timer);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutData("Alliance Color", &m_alliance);
  alliance_color = "red";  // Default evaluated in auto and teleop inits
  turret_direction = true; // Initial turrent scan direction
  shooter_trim = 0;
  frc::SmartDashboard::PutNumber("Shooter Trim", shooter_trim);

  frc::SmartDashboard::PutBoolean("Climber reset", false); 
// Initial pnematic states
  MyAppendage.Intake_Up();
  MyDrive.climber_hold();
  MyDrive.climber_tiltin();
  MyAppendage.Articulate(4);

  // Dashboard input creations
  MyAppendage.DashboardCreate();
  MyDrive.DashboardCreate();
  intakedelay = 0;
  intakedelay2 = 0;
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
  FourBallSecondTime = false;
  counter2 = 0;
  state_drive = 0;
  MyDrive.reset_drive_s();


  // 4ball auto stuff
  int FirstSectionOffset = 50*6; // Gives 6sec for first part of auto to take place
  int SecondSelectionOffset = 0;

  // 2 ball stuff
  int ct_off = 0;

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
  MyAppendage.controlpanel_colorsense_init();
}

void Robot::AutonomousPeriodic(){

  m_autoSelected = m_chooser.GetSelected();
  //Compressor Code
  compressor.EnableAnalog(units::pounds_per_square_inch_t(85), units::pounds_per_square_inch_t (120));

  //Reset shooter variables
  bool align = false;
  bool atspeed = false;
  char ball_color = MyAppendage.controlpanel_colorsense_periodic();



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
  bool moved = false;
  if (counter >= auto_timer) {

    if (m_autoSelected == kAutoName2Ball){
      // 2 Ball Autonomous

      if ((counter - auto_timer) <= 20){
        MyAppendage.Intake_Down();
        MyAppendage.Intake_In();
        MyDrive.camera_intake(intake_camera_x, 0);
        moved = false;
      }
      else if (counter <= (150 + auto_timer) ){
        MyDrive.camera_intake(intake_camera_x, -0.5);
        MyAppendage.Intake_Down();
        tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false, true);
        bool LightGate_val = MyAppendage.Intake_In();
        moved = true;
      }
            else if (counter <= (270 + auto_timer) ){
        MyDrive.Joystick_Drive(0,0);
        MyAppendage.Intake_Up();
        MyAppendage.Intake_Off();
        //double distance = MyAppendage.Get_Distance(shooter_camera_y);
        tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false, false);
        MyAppendage.Articulate(distance);
        atspeed = MyAppendage.Shooter_Encoder_distance(distance,shooter_trim);
        moved = true;
            }

      else if (counter <= (570 + auto_timer)){
        auto_ball_pickedup = true;

        //double distance = MyAppendage.Get_Distance(shooter_camera_y);
        tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false, false);
        MyAppendage.Articulate(distance);
        atspeed = MyAppendage.Shooter_Encoder_distance(distance,shooter_trim);
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
      else{
        MyAppendage.Shooter_Off();
        MyAppendage.Feeder_Off();
        MyAppendage.Intake2_Off();
        MyAppendage.Rotate_Off();
        MyDrive.Joystick_Drive(0,0);

      }
    }
    // End of 2ball auto
//TEST path 

  else if (m_autoSelected == kAutoNameCustom2){
      bool reached_distance;
      bool reached_angle;

      if (counter <= 150){
       reached_angle = MyDrive.turnto_gyro(18);
        //if (reached_angle){
          //state_drive++;
          //MyDrive.reset_drive_s();
       // }
      
     // else if (state_drive == 1){
        // reached_distance = MyDrive.driveto_distance(-60);
        /*if (reached_distance){
            state_drive++;
            MyDrive.reset_drive_s();
          }*/
      }
      else{
        MyDrive.Joystick_Drive(0,0);
      }

    }

// End of test
///////////////////////////////////////////////////////////////////////////////
   else if (m_autoSelected == kAutoName4BallNoPath){
      // 4 Ball Autonomous No Path Planning
      // Delay doesn't work

      if (counter < 20){
        MyAppendage.Intake_Down();
        MyAppendage.Intake_In();
        MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x -5, turret_direction, false, false, true);
        MyDrive.camera_intake(intake_camera_x, 0);
        moved = false;
      }

      else if (counter <= 100 || (FourBallSecondTime && counter2 < 290)){

        
        MyAppendage.Intake_Down();
        MyAppendage.Intake_In();
        MyAppendage.Shooter_Off();
        MyAppendage.Feeder_Off();
        MyAppendage.Intake2_Off();
        MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, true, false, false);
        intakedelay = 0;
        moved = true;

        if (FourBallSecondTime){
         /* if (counter2 <= 35){
            if(counter2==0){
              MyDrive.reset_drive_s();
            }
           MyDrive.turnto_gyro(0);
            }
          */

          if (counter2 <= 235){
            if(counter2==0){
              MyDrive.reset_drive_s();
            }
        
        bool LightGate_val = MyAppendage.Intake_In(ball_color);

          if (LightGate_val){
            MyAppendage.Intake2_In();
          }
          else{
            MyAppendage.Intake2_Off();
            //frc::SmartDashboard::PutString("Intake State", "Off");

  }

            if (counter2 >= 135 && intake_camera_exist == 1){
                MyDrive.camera_intake(intake_camera_x, 0.7);
            }

              else{

           MyDrive.driveto_distance(290);

              }
            
          }
    
          else if(counter2<=240){
            MyDrive.Joystick_Drive(0,0);
          }
          else if (counter2 <= 290){
            if(counter2==241){
              MyDrive.reset_drive_s();
            }
           MyDrive.driveto_distance(-250);
            
          }
          else{
            MyDrive.Joystick_Drive(0,0);
          }
          
          counter2 ++;
        }
        else{
          MyDrive.camera_intake(intake_camera_x, -0.5);
        }

      }
      else if (counter <= 175 || (FourBallSecondTime && counter2 <= 310)){

            MyDrive.Joystick_Drive(0,0);
            tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false, false);
            MyAppendage.Articulate(distance);
            atspeed = MyAppendage.Shooter_Encoder_distance(distance,shooter_trim);

            counter2 ++;
        }
      else if (counter < 250 || FourBallSecondTime){
        auto_ball_pickedup = true;
        if (intakedelay < 10){
            MyAppendage.Intake_In();
          }
          else{
            MyAppendage.Intake_Off();
          }
          intakedelay ++;
          if (intakedelay > 500){
            intakedelay = 30;
          }
        MyAppendage.Intake_Up();
        //double distance = MyAppendage.Get_Distance(shooter_camera_y);
        tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false, false);
        MyAppendage.Articulate(distance);
        atspeed = MyAppendage.Shooter_Encoder_distance(distance,shooter_trim);
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
      else{
        counter2 = 0;
        auto_ball_pickedup = false;
        FourBallSecondTime = true;
      }
    }  
    //end 4 ball auto
    else if (m_autoSelected == kAutoName3BallPath){
     // 3 Ball Autonomous no auto timer
      if (counter <= 20){
        MyAppendage.Intake_Down();
        MyAppendage.Intake_In();
        MyDrive.camera_intake(intake_camera_x, 0);
        moved = false;
      }
      else if (counter <= 100 ){
        MyDrive.camera_intake(intake_camera_x, -0.5);
        MyAppendage.Intake_Down();
        tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false, true);
        bool LightGate_val = MyAppendage.Intake_In();
        moved = true;
      }
        else if (counter <= 175 ){
        MyDrive.Joystick_Drive(0,0);
        MyAppendage.Intake_Down();
         MyAppendage.Intake_In();
        //double distance = MyAppendage.Get_Distance(shooter_camera_y);
        tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false, false);
        MyAppendage.Articulate(distance);
        atspeed = MyAppendage.Shooter_Encoder_distance(distance,shooter_trim);
        moved = true;
        }

      else if (counter <= 250 ){
        auto_ball_pickedup = true;
       
        //double distance = MyAppendage.Get_Distance(shooter_camera_y);
        tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false, false);
        MyAppendage.Articulate(distance);
        atspeed = MyAppendage.Shooter_Encoder_distance(distance,shooter_trim);
        MyDrive.Joystick_Drive(0,0);

        if (align && atspeed){
          MyAppendage.Feeder_In();
          MyAppendage.Intake2_In();
        }
        else{
          MyAppendage.Feeder_Off();
          MyAppendage.Intake2_Off();
        }
      } else if (counter <= 300) {
        //Drive to next ball.

        MyAppendage.Shooter_Off();
        MyAppendage.Rotate_Off();
        MyAppendage.Feeder_Off();
        MyAppendage.Intake2_Off();

        if (counter == 251){
          MyDrive.reset_drive_s();
        }
        tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, true, false, false);

        MyDrive.driveto_distance(-40);
      }
      else if (counter <= 350) {
        //Drive to next ball.

        if (counter == 301){
          MyDrive.reset_drive_s();
        }
        tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, true, false, false);
        MyDrive.turnto_gyro(110);
      }
      else if (counter <= 425) {
        //Drive to next ball.

        if (counter == 351){
          MyDrive.reset_drive_s();
        }
      
        MyDrive.driveto_distance(175);
        MyAppendage.Intake_In();
        MyAppendage.Intake_Down();
      }
      else if (counter <= 475) {
        //Drive to next ball.

        if (counter == 426){
          MyDrive.reset_drive_s();
        }

        MyDrive.turnto_gyro(-45);
      }
    /*  else if (counter <= 450 ){
        MyDrive.camera_intake(intake_camera_x, -0.5);
        MyAppendage.Intake_Down();
        tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false, true);
        bool LightGate_val = MyAppendage.Intake_In();
        moved = true;
      }
*/
      else if (counter <= 550){
        MyDrive.Joystick_Drive(0,0);
        MyAppendage.Intake_Up();
        MyAppendage.Intake_Off();
        //double distance = MyAppendage.Get_Distance(shooter_camera_y);
        tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false, false);
        MyAppendage.Articulate(distance);
        atspeed = MyAppendage.Shooter_Encoder_distance(distance,shooter_trim);
        moved = true;
        }

      else if (counter <= 750 ){
        auto_ball_pickedup = true;
        

        //double distance = MyAppendage.Get_Distance(shooter_camera_y);
        tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false, false);
        MyAppendage.Articulate(distance);
        atspeed = MyAppendage.Shooter_Encoder_distance(distance,shooter_trim);
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
        
      else{
        MyAppendage.Shooter_Off();
        MyAppendage.Feeder_Off();
        MyAppendage.Intake2_Off();
        MyAppendage.Rotate_Off();
        MyDrive.Joystick_Drive(0,0);

      }

    } // end 3ball auto
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
  trim_state = false;
  intake_toomany = 0;


  MyAppendage.controlpanel_colorsense_init();

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
  //frc::SmartDashboard::PutString("Alliance",alliance_color);
}
void Robot::TeleopPeriodic(){
  char ball_color = MyAppendage.controlpanel_colorsense_periodic();
  int ballCnt = MyAppendage.BallCounter(ball_color);
  frc::SmartDashboard::PutNumber("Ball Count",ballCnt);

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

  bool climber_reset = frc::SmartDashboard::GetBoolean("Climber reset", false);
  if (climber_reset) {
    if (c1_lefttrigger > 0.5 || c1_leftbmp || c1_righttrigger > 0.5 || c1_rightbmp) {
      if (c1_leftbmp){
        MyDrive.climber_extend_nolimit_left();

      } else if (c1_lefttrigger > 0.5) {
        MyDrive.climber_retract_nolimit_left();
      } else {
        MyDrive.climber_off_left();
      }
      if (c1_rightbmp){
        MyDrive.climber_extend_nolimit_right();

      } else if (c1_righttrigger > 0.5) {
        MyDrive.climber_retract_nolimit_right();
      } else {
        MyDrive.climber_off_right();
      }
    } else {
      MyDrive.climber_hold();
    }
  }
  else if (endgame_unlock){
    bool output;
    bool output_1;
    
    // Extend / Retract Arms
    if (c1_righttrigger > 0.5 && !c1_btn_back)
    {
      MyDrive.climber_retract();
    }
    
    else if ( c1_btn_start && c1_lefttrigger >0.5){
      MyDrive.climber_extend_first();
    }

    else if (c1_lefttrigger > 0.5 && !c1_btn_back)
    {
      MyDrive.climber_extend();
      
    }

    
    else if (c1_righttrigger > 0.5 && c1_btn_back)
    {
      MyDrive.climber_retract_nolimit();
    }

    else if (c1_lefttrigger > 0.5 && c1_btn_back)
    {
      MyDrive.climber_extend_nolimit();
      
    }

    else if (c1_btn_x){ //Auto climb

      switch (climber_state){
        case 0:
          output = MyDrive.climber_retract();
            
            climber_count = 0;

            if (output == 1){
              climber_state ++;
            }
            break;
            case 1:
              MyDrive.climber_count_reset();
              climber_state ++;
            break;
            case 2:
          
           output = MyDrive.climber_extend();
            if (output==2){
              climber_state ++;
            }
            break;
            case 3:
            output = MyDrive.climber_extend();
            
            MyDrive.climber_tiltout();

            if (output == 1){
              climber_state ++;
            }
            break;

            case 4:
              MyDrive.climber_count_reset();
              climber_state ++;
            break;

            case 5:
            MyDrive.climber_tiltin();
            output = MyDrive.climber_extend();
            if (output == 1 && climber_count >= 15){
              climber_state ++;
            }
              climber_count ++;
              break;

            case 6:
              MyDrive.climber_count_reset();
              climber_state=0;
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
    intake_toomany = 0;
    MyAppendage.Intake_Down();
    intakedelay = 0;

    bool LightGate_val = MyAppendage.Intake_In(ball_color);
    

  if (LightGate_val && !shooter_test){
    MyAppendage.Intake2_In();
  }
  else{
    MyAppendage.Intake2_Off();
    //frc::SmartDashboard::PutString("Intake State", "Off");
  }
}
else if(c2_btn_y || ballCnt == 3 || intake_toomany > 0){ //comment this back in when we are ready to do auto 3 ball reject
  
  if (ballCnt == 3){
    intake_toomany = 25;
  }

  intake_toomany --;

  if (intakedelay2 < 10){
    MyAppendage.Intake_In(ball_color);
  }
  else{MyAppendage.Intake_Out();
  }
  MyAppendage.Intake_Down();
  
  intakedelay2++;
}
else{
  if (intakedelay < 10){
    MyAppendage.Intake_In(ball_color);
  }
  else{
    MyAppendage.Intake_Off();
    //MyAppendage.Intake2_Off();
  }
  intakedelay ++;
  intakedelay2 = 0;
  if (intakedelay > 500){
    intakedelay = 30;
  }
  MyAppendage.Intake_Up();
  
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
  if(!trim_state){
    shooter_trim ++;
  }
  
  trim_state = true;
}
else if(c2_dpad > 260 && c2_dpad < 280){ // Left on dpad
  if(!trim_state){
    shooter_trim --;
  }
  trim_state = true;
}
else{
  trim_state = false;
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
  MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, true, false);
  //MyAppendage.Rotate_Off(); // Only for testing, line above should be used for competition.
  MyAppendage.Shooter_Off();
  MyAppendage.Feeder_Off();
  MyAppendage.Intake2_Off();
  MyAppendage.Articulate(140);
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

  tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, true, false, false);

  atspeed = MyAppendage.Shooter_Encoder_distance(1, 0);
  MyAppendage.Articulate(120); //harcode for close shot

  if(c2_right_trigger > 0.5){ // Shoot ball
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

  atspeed = MyAppendage.Shooter_Encoder_distance(1, 0);

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

  //tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false);

  //High Fixed shoot

  tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, true, false, false);

  atspeed = MyAppendage.Shooter_Encoder_distance(170,shooter_trim);
  MyAppendage.Articulate(144); //harcode for far shot

  if(c2_right_trigger > 0.5){ // Shoot ball
    MyAppendage.Feeder_In();
    MyAppendage.Intake2_In();
  }
  else{
      MyAppendage.Feeder_Off();
      MyAppendage.Intake2_Off();
  }

}


else {
  
  if (c2_left_trigger >= 0.5)
  {
    //Get shooter aligned and up to speed
    atspeed = MyAppendage.Shooter_Encoder_distance(distance,shooter_trim);
    MyAppendage.Articulate(distance);
    frc::SmartDashboard::PutBoolean("Alligned", align);
    frc::SmartDashboard::PutBoolean("AtSpeed", atspeed);

  if((alliance_color == "blue" && ball_color == 'R') || (alliance_color == "red" && ball_color == 'B')){
      tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x + 5, turret_direction, false, false, false);

  }
  else{
    tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, false, false, false);

  }

    if(align && atspeed && (c2_right_trigger > 0.5)){ // Shoot ball
      MyAppendage.Feeder_In();
      MyAppendage.Intake2_In();
    }
    else{
      MyAppendage.Feeder_Off();
      MyAppendage.Intake2_Off();
    }

  }


  else if((alliance_color == "blue" && ball_color == 'R') || (alliance_color == "red" && ball_color == 'B')){
  
  atspeed = MyAppendage.Shooter_Encoder_distance(1, 0);

  if( atspeed ){ // Shoot ball
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
   // MyAppendage.Rotate_Off();
       tie(align,turret_direction) = MyAppendage.Rotate(shooter_camera_exist, shooter_camera_x, turret_direction, true, false, false);

    
    if(!c2_leftbumper){
      MyAppendage.Intake2_Off();
      MyAppendage.Feeder_Off();
    }
    

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

MyLog.CurrentCompare(19, 7);
MyLog.CurrentCompare(18, 8);
MyLog.CurrentCompare(0, 9);
MyLog.CurrentCompare(1, 10);

//Shooter Current Compares

MyLog.CurrentCompare(13, 14);
MyLog.CurrentCompare(14, 2);


MyLog.Dashboard();
MyLog.PDPTotal();
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
