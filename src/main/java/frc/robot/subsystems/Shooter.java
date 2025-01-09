package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants;

public class Shooter extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Shooter mInstance;
  private PeriodicIO mPeriodicIO;

  public static Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
  }

  private SparkFlex mLeftShooterMotor;
  private SparkFlex mRightShooterMotor;

  private RelativeEncoder mLeftShooterEncoder;
  private RelativeEncoder mRightShooterEncoder;

  private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);

  private Shooter() {
    super("Shooter");

    mPeriodicIO = new PeriodicIO();

    mLeftShooterMotor = new SparkFlex(Constants.kShooterLeftMotorId, MotorType.kBrushless);
    mRightShooterMotor = new SparkFlex(Constants.kShooterRightMotorId, MotorType.kBrushless);

    var shooterConfig = new SparkFlexConfig()
      .idleMode(IdleMode.kCoast);

    shooterConfig
      .closedLoop
        .pidf(Constants.kShooterP, Constants.kShooterI, Constants.kShooterD, Constants.kShooterFF)
        .minOutput(Constants.kShooterMinOutput)
        .maxOutput(Constants.kShooterMaxOutput);

    var leftShooterConfig = new SparkFlexConfig()
      .apply(shooterConfig)
      .inverted(true);

    mLeftShooterMotor.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mRightShooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mLeftShooterEncoder = mLeftShooterMotor.getEncoder();
    mRightShooterEncoder = mRightShooterMotor.getEncoder();
  }

  private static class PeriodicIO {
    double shooter_rpm = 0.0;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
    double limitedSpeed = mSpeedLimiter.calculate(mPeriodicIO.shooter_rpm);
    mLeftShooterMotor.getClosedLoopController().setReference(limitedSpeed, ControlType.kVelocity);
    mRightShooterMotor.getClosedLoopController().setReference(limitedSpeed,ControlType.kVelocity);
  }

  @Override
  public void stop() {
    stopShooter();
  }

  @Override
  public void outputTelemetry() {
    putNumber("Speed (RPM):", mPeriodicIO.shooter_rpm);
    putNumber("Left speed:", mLeftShooterEncoder.getVelocity());
    putNumber("Right speed:", mRightShooterEncoder.getVelocity());
  }

  @Override
  public void reset() {
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void setSpeed(double rpm) {
    mPeriodicIO.shooter_rpm = rpm;
  }

  public void stopShooter() {
    mPeriodicIO.shooter_rpm = 0.0;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
