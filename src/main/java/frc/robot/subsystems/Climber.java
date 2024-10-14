package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

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

  private CANSparkMax mLeftClimberMotor;
  private CANSparkMax mRightClimberMotor;

  private SparkPIDController mLeftClimberPID;
  private SparkPIDController mRightClimberPID;

  private RelativeEncoder mLeftClimberEncoder;
  private RelativeEncoder mRightClimberEncoder;

  private Climber() {
    super("Climber");

    mPeriodicIO = new PeriodicIO();

    mLeftClimberMotor = new CANSparkMax(Constants.kClimberLeftMotorId, MotorType.kBrushless);
    mRightClimberMotor = new CANSparkMax(Constants.kClimberRightMotorId, MotorType.kBrushless);

    mLeftClimberPID = mLeftClimberMotor.getPIDController();
    mLeftClimberPID.setP(Constants.kClimberP);
    mLeftClimberPID.setI(Constants.kClimberI);
    mLeftClimberPID.setD(Constants.kClimberD);
    mLeftClimberPID.setOutputRange(Constants.kClimberMinOutput, Constants.kClimberMaxOutput);

    mRightClimberPID = mRightClimberMotor.getPIDController();
    mRightClimberPID.setP(Constants.kClimberP);
    mRightClimberPID.setI(Constants.kClimberI);
    mRightClimberPID.setD(Constants.kClimberD);
    mRightClimberPID.setOutputRange(Constants.kClimberMinOutput, Constants.kClimberMaxOutput);

    mLeftClimberEncoder = mLeftClimberMotor.getEncoder();
    mLeftClimberEncoder.setPositionConversionFactor(Constants.kClimberGearRatio);
    mLeftClimberEncoder.setVelocityConversionFactor(Constants.kClimberGearRatio);

    mRightClimberEncoder = mRightClimberMotor.getEncoder();
    mRightClimberEncoder.setPositionConversionFactor(Constants.kClimberGearRatio);
    mRightClimberEncoder.setVelocityConversionFactor(Constants.kClimberGearRatio);

    mLeftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mRightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    mLeftClimberMotor.setInverted(false);
    mRightClimberMotor.setInverted(true);
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
    mLeftClimberPID.setReference(mPeriodicIO.climber_left_speed, ControlType.kVelocity);
    mRightClimberPID.setReference(mPeriodicIO.climber_right_speed, ControlType.kVelocity);
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
    mLeftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mRightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setCoastMode() {
    mLeftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mRightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
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
