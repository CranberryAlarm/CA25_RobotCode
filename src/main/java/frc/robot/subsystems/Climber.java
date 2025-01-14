package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class Climber extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Climber mInstance;
  private PeriodicIO mPeriodicIO;

  public static Climber getInstance() {
    if (mInstance == null) {
      mInstance = new Climber();
    }
    return mInstance;
  }

  private SparkMax mLeftClimberMotor;
  private SparkMax mRightClimberMotor;

  private RelativeEncoder mLeftClimberEncoder;
  private RelativeEncoder mRightClimberEncoder;

  private Climber() {
    super("Climber");

    mPeriodicIO = new PeriodicIO();

    mLeftClimberMotor = new SparkMax(Constants.kClimberLeftMotorId, MotorType.kBrushless);
    mRightClimberMotor = new SparkMax(Constants.kClimberRightMotorId, MotorType.kBrushless);

    var climberConfig = new SparkMaxConfig();
    climberConfig.closedLoop.pid(Constants.kClimberP, Constants.kClimberI, Constants.kClimberD)
        .minOutput(Constants.kClimberMinOutput)
        .maxOutput(Constants.kClimberMaxOutput);

    climberConfig.encoder.positionConversionFactor(Constants.kClimberGearRatio)
        .velocityConversionFactor(Constants.kClimberGearRatio);

    climberConfig.idleMode(IdleMode.kBrake);

    var climberRightConfig = new SparkMaxConfig()
        .apply(climberConfig)
        .inverted(true);

    mLeftClimberMotor.configure(climberConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    mRightClimberMotor.configure(climberRightConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    mLeftClimberEncoder = mLeftClimberMotor.getEncoder();
    mRightClimberEncoder = mRightClimberMotor.getEncoder();
  }

  private static class PeriodicIO {
    double climber_right_speed = 0.0; // RPM of spool (Spark Max default unit)
    double climber_left_speed = 0.0; // RPM of spool (Spark Max default unit)
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
    mLeftClimberMotor.getClosedLoopController().setReference(mPeriodicIO.climber_left_speed, ControlType.kVelocity);
    mRightClimberMotor.getClosedLoopController().setReference(mPeriodicIO.climber_right_speed, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    stopClimber();
  }

  @Override
  public void outputTelemetry() {
    putNumber("Left speed setpoint:", mPeriodicIO.climber_left_speed);
    putNumber("Left speed:", mLeftClimberEncoder.getVelocity());
    putNumber("Right speed setpoint:", mPeriodicIO.climber_right_speed);
    putNumber("Right speed:", mRightClimberEncoder.getVelocity());
  }

  @Override
  public void reset() {
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void setBrakeMode() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    mLeftClimberMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    mRightClimberMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setCoastMode() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    mLeftClimberMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    mRightClimberMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void climb() {
    mPeriodicIO.climber_left_speed = Constants.kClimberClimbSpeed;
    mPeriodicIO.climber_right_speed = Constants.kClimberClimbSpeed;
  }

  public void release() {
    mPeriodicIO.climber_left_speed = Constants.kClimberReleaseSpeed;
    mPeriodicIO.climber_right_speed = Constants.kClimberReleaseSpeed;
  }

  public void tiltLeft() {
    mPeriodicIO.climber_left_speed = Constants.kClimberReleaseSpeed;
    mPeriodicIO.climber_right_speed = 0.0;
  }

  public void tiltRight() {
    mPeriodicIO.climber_left_speed = 0.0;
    mPeriodicIO.climber_right_speed = Constants.kClimberReleaseSpeed;
  }

  public void stopClimber() {
    mPeriodicIO.climber_left_speed = 0.0;
    mPeriodicIO.climber_right_speed = 0.0;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
