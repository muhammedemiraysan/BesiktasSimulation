package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class ArmSubsystem extends SubsystemBase implements AutoCloseable {

  private double m_armSetpointDegrees = Constants.kDefaultArmSetpointDegrees;
  
  private final DCMotor m_armGearbox = DCMotor.getFalcon500(1);

  final int kCountsPerRev = 4096;
  final double kSensorGearRatio = 1;
  final double kGearRatio = 1;
  final double kWheelRadiusInches = 3;
  final int k100msPerSecond = 10;

  private final WPI_TalonFX _talon = new WPI_TalonFX(1);
  private final AnalogPotentiometer m_wristPot = new AnalogPotentiometer(1, 180);

  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          m_armGearbox,
          Constants.kArmReduction,
          SingleJointedArmSim.estimateMOI(Constants.kArmLength, Constants.kArmMass),
          Constants.kArmLength,
          Constants.kMinAngleRads,
          Constants.kMaxAngleRads,
          true,
          VecBuilder.fill(Constants.kArmEncoderDistPerPulse)
          );
          
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(
        new MechanismLigament2d(
          "ArmTower",
          30, 
          -90));
  
    private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));
  
  private final TalonFXSimCollection fx_sim = _talon.getSimCollection();
  
  public ArmSubsystem() {

		_talon.configFactoryDefault();
		_talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
				Constants.kTimeoutMs);
		_talon.configNeutralDeadband(0.001, Constants.kTimeoutMs);
		_talon.setSensorPhase(false);
		_talon.setInverted(false);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		_talon.selectProfileSlot(Constants.kSlotIdx, 0);
		_talon.config_kF(Constants.kSlotIdx, Constants.kF, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kSlotIdx, Constants.kP, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kSlotIdx, Constants.kI, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kSlotIdx, Constants.kD, Constants.kTimeoutMs);
		_talon.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		_talon.configMotionAcceleration(6000, Constants.kTimeoutMs);
		_talon.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    
    SmartDashboard.putData("Arm Sim", m_mech2d);
    
    m_armTower.setColor(new Color8Bit(Color.kRed));
  }

  public void simulationPeriodic() {

    m_armSetpointDegrees = m_wristPot.get();

    m_armSim.setInput(fx_sim.getMotorOutputLeadVoltage() * RobotController.getBatteryVoltage());
    m_armSim.update(0.020);

    _talon.setSelectedSensorPosition(Units.radiansToDegrees(m_armSim.getAngleRads()));

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

    SmartDashboard.putNumber("m_armSetpointDegress", m_armSetpointDegrees);
    SmartDashboard.putNumber("outputVoltage", _talon.getMotorOutputVoltage());

  }

  public void reachSetpoint() {
    _talon.set(TalonFXControlMode.MotionMagic, m_armSetpointDegrees);
  }

  public void stop() {
    _talon.set(0.0);
  }

  @Override
  public void close() {
    _talon.close();
    m_mech2d.close();
    m_armPivot.close();
    m_arm.close();
  }
}