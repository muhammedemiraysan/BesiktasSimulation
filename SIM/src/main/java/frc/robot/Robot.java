// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** This is a sample program to demonstrate the use of arm simulation with existing code. */

public class Robot extends TimedRobot {
  private final RobotContainer m_robot = new RobotContainer();
  @Override
  public void robotInit() {
    m_robot.configureButtonBindings();
  }
  
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void close() {
    super.close();
  }
}