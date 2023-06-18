package frc.robot;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static final int kMotorPort = 0;
  public static final int kJoystickPort = 0;

  public static final String kArmPositionKey = "ArmPosition";
  //smartDasboard için isim etiketi

  public static final double kP = 1.3;
  public static final double kI = 0.001;
  public static final double kD = 0.8;
  public static final double kF = 1.5;
  public static final int kSlotIdx = 0;
  public static final int kTimeoutMs = 100;
  //PID değişkenleri

  public static final double kDefaultArmSetpointDegrees = 150.0;

  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  public static final double kArmReduction = 10;
  public static final double kArmMass = 5.0; // Kilograms
  public static final double kArmLength = Units.inchesToMeters(30);
  public static final double kMinAngleRads = Units.degreesToRadians(-75);
  public static final double kMaxAngleRads = Units.degreesToRadians(255);
  //SingleJointedArmSim değişkenleri
}