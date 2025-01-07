package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.simulation.SimulatableCANSparkMax;
import frc.robot.wrappers.REVThroughBoreEncoder;

public class Algae extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Algae mInstance;
  private PeriodicIO mPeriodicIO;

  public static Algae getInstance() {
    if (mInstance == null) {
      mInstance = new Algae();
    }
    return mInstance;
  }

  private SimulatableCANSparkMax mWristMotor;
  private final ProfiledPIDController mWristPIDController;
  private final ArmFeedforward mWristFeedForward;

  private SimulatableCANSparkMax mIntakeMotor;

  private final REVThroughBoreEncoder mWristAbsEncoder = new REVThroughBoreEncoder(Constants.Algae.kWristEncoderId);

  private Algae() {
    super("Algae");

    mPeriodicIO = new PeriodicIO();

    // WRIST
    mWristMotor = new SimulatableCANSparkMax(Constants.Algae.kWristMotorId, MotorType.kBrushless);
    mWristMotor.restoreFactoryDefaults();
    mWristMotor.setIdleMode(IdleMode.kCoast);
    mWristMotor.setSmartCurrentLimit(Constants.Algae.kMaxWristCurrent);

    // Wrist PID
    mWristPIDController = new ProfiledPIDController(
        Constants.Algae.kWristP,
        Constants.Algae.kWristI,
        Constants.Algae.kWristD,
        new TrapezoidProfile.Constraints(
            Constants.Algae.kWristMaxVelocity,
            Constants.Algae.kWristMaxAcceleration));

    // Wrist Feedforward
    mWristFeedForward = new ArmFeedforward(
        Constants.Algae.kWristKS,
        Constants.Algae.kWristKG,
        Constants.Algae.kWristKV,
        Constants.Algae.kWristKA);

    // INTAKE
    mIntakeMotor = new SimulatableCANSparkMax(Constants.Algae.kIntakeMotorId, MotorType.kBrushless);
    mIntakeMotor.restoreFactoryDefaults();
    mIntakeMotor.setIdleMode(IdleMode.kBrake);
    mIntakeMotor.setSmartCurrentLimit(Constants.Algae.kMaxIntakeCurrent);

    mIntakeMotor.setInverted(true);

    mWristMotor.setInverted(true);

    mWristMotor.burnFlash();
    mIntakeMotor.burnFlash();
  }

  private static class PeriodicIO {
    double wrist_target_angle = 0.0;
    double wrist_voltage = 0.0;

    double intake_power = 0.0;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    double pidCalc = mWristPIDController.calculate(getWristAngle(), mPeriodicIO.wrist_target_angle);
    double ffCalc = mWristFeedForward.calculate(Math.toRadians(getWristReferenceToHorizontal()),
        Math.toRadians(mWristPIDController.getSetpoint().velocity));

    mPeriodicIO.wrist_voltage = pidCalc + ffCalc;
  }

  @Override
  public void writePeriodicOutputs() {
    mWristMotor.set(mPeriodicIO.wrist_voltage);

    // mIntakeMotor.set(mPeriodicIO.intake_power);
    mIntakeMotor.setVoltage(mPeriodicIO.intake_power);
  }

  @Override
  public void stop() {
    mPeriodicIO.wrist_voltage = 0.0;
    mPeriodicIO.wrist_target_angle = Constants.Algae.kStowAngle;

    mWristMotor.set(0.0);
    mIntakeMotor.set(0.0);
  }

  @Override
  public void outputTelemetry() {
    putNumber("Wrist/Position", getWristAngle());
    putNumber("Wrist/Target", mPeriodicIO.wrist_target_angle);
    putNumber("Wrist/Current", mWristMotor.getOutputCurrent());
    putNumber("Wrist/Output", mWristMotor.getAppliedOutput());
    putNumber("Wrist/Voltage", mPeriodicIO.wrist_voltage);
    putNumber("Wrist/Frequency", mWristAbsEncoder.getFrequency());

    putNumber("Intake/Current", mIntakeMotor.getOutputCurrent());
    putNumber("Intake/Output", mIntakeMotor.getAppliedOutput());
    putNumber("Intake/Power", mPeriodicIO.intake_power);
  }

  @Override
  public void reset() {
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void stow() {
    mPeriodicIO.wrist_target_angle = Constants.Algae.kStowAngle;
    mPeriodicIO.intake_power = 0.0;
  }

  public void grabAlgae() {
    mPeriodicIO.wrist_target_angle = Constants.Algae.kDeAlgaeAngle;
    mPeriodicIO.intake_power = Constants.Algae.kIntakeSpeed;
  }

  public void score() {
    mPeriodicIO.intake_power = Constants.Algae.kEjectSpeed;
  }

  public void groundIntake() {
    mPeriodicIO.wrist_target_angle = Constants.Algae.kGroundIntakeAngle;
    mPeriodicIO.intake_power = Constants.Algae.kGroundIntakeSpeed;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  public double getWristAngle() {
    return Units.rotationsToDegrees(mWristAbsEncoder.getAbsolutePosition());
  }

  public double getWristReferenceToHorizontal() {
    return getWristAngle() - Constants.Algae.kWristOffset;
  }
}
