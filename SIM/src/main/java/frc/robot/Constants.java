// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort = 0;

  public static final String kArmPositionKey = "ArmPosition";

  public static final String kArmPKey = "ArmP";

<<<<<<< Updated upstream
  // The P gain for the PID controller that drives this arm.
  public static final double kDefaultArmKp = 0.4;
  public static final double kDefaultArmKi = 0.1;
  public static final double kDefaultArmKd = 0.3;
  public static final double kDefaultArmKf = 0.1;

=======
  public static final double kP = 0.1;
  public static final double kI = 0.0015;
  public static final double kD = 0;
  public static final double kF = 1;
  public static final int kSlotIdx = 0;
  public static final int kTimeoutMs = 1;
>>>>>>> Stashed changes
  public static final double kDefaultArmSetpointDegrees = 150.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  public static final double kArmReduction = 10;
  public static final double kArmMass = 15.0; // Kilograms
  public static final double kArmLength = Units.inchesToMeters(30);
  public static final double kMinAngleRads = Units.degreesToRadians(-75);
  public static final double kMaxAngleRads = Units.degreesToRadians(255);
}