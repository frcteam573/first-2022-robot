// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include "Drive.h"
#include "Appendage.h"
#include "Led.h"
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include "frc/DriverStation.h"
#include "frc/Compressor.h"
#include "Log.h"



class Robot : public frc::TimedRobot {

 public:

  //Include subsystem object definitions here
  Drive MyDrive; 
  Appendage MyAppendage;
  Led MyLed;
  Log MyLog;

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;


 private:

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Drive Back";
  const std::string kAutoNameCustom = "2 Ball shoot";
  const std::string kAutoNameCustom1 = "4 Ball Shoot";

  std::string m_autoSelected;

  frc::Compressor compressor{1, frc::PneumaticsModuleType::REVPH};

  frc::Joystick controller1{0}; // Driver controller
  frc::Joystick controller2{1}; // Operator controller

  bool drive_straight_first;
  string alliance_color; // Hold current alliance color for auto ball pickup
  bool endgame_unlock;  // Lock to prevent accidental climber deployment
  bool turret_direction; // Hold current turret direction to prevent jumping when scanning
  int counter; 
  int climber_state;
  int climber_count;
 
};
