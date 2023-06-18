package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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