package frc.robot;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final ArmSubsystem m_robotArm = new ArmSubsystem();

  CommandXboxController m_driverController =
      new CommandXboxController(Constants.kJoystickPort);

  public RobotContainer() {
    configureButtonBindings();
  }
  public void configureButtonBindings() {
    m_driverController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotArm.reachSetpoint();                
                },
                m_robotArm));
    m_driverController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                    m_robotArm.stop();                
                  },
                  m_robotArm));
  }
}