package frc.robot.subsystems;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.PIDConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import frc.robot.Constants;

public class Coral extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Coral mInstance;
  private PeriodicIO mPeriodicIO;

  public static Coral getInstance() {
    if (mInstance == null) {
      mInstance = new Coral();
    }
    return mInstance;
  }

  private ThriftyNova mLeftMotor;
  private ThriftyNova mRightMotor;
  private PIDConfig mPIDConfig;

  private LaserCan mLaserCAN;
  private int mLaserReading;

  private Coral() {
    super("Coral");

    mPeriodicIO = new PeriodicIO();

    mLeftMotor = new ThriftyNova(Constants.Coral.kLeftMotorId);
    mRightMotor = new ThriftyNova(Constants.Coral.kRightMotorId);

    // mLeftMotor.factoryReset();
    // mRightMotor.factoryReset();

    mLeftMotor.setBrakeMode(false);
    mRightMotor.setBrakeMode(false);

    // mLeftMotor.setMaxCurrent(CurrentType.STATOR, Constants.Coral.kMaxCurrent);
    // mRightMotor.setMaxCurrent(CurrentType.STATOR, Constants.Coral.kMaxCurrent);

    // mLeftMotor.setMaxCurrent(CurrentType.STATOR, 200.0);
    // mRightMotor.setMaxCurrent(CurrentType.STATOR, 200.0);

    // mLeftMotor.setMaxCurrent(CurrentType.SUPPLY, 200.0);
    // mRightMotor.setMaxCurrent(CurrentType.SUPPLY, 200.0);

    mLeftMotor.setRampUp(0.5);
    mRightMotor.setRampUp(0.5);

    mLeftMotor.setMaxOutput(1.0);
    mRightMotor.setMaxOutput(1.0);

    mPIDConfig = mRightMotor.pid0;
    mPIDConfig.setP(Constants.Coral.kP);
    mPIDConfig.setI(Constants.Coral.kI);
    mPIDConfig.setD(Constants.Coral.kD);
    mPIDConfig.setIZone(Constants.Coral.kIZone);

    // TODO: Make sure we're inverting the correct motor
    // mLeftMotor.setInverted(true);
    // mLeftMotor.setInversion(false);
    // mRightMotor.setInversion(false);

    mLaserCAN = new LaserCan(Constants.Coral.kLaserId);
    try {
      mLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      mLaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      mLaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  private static class PeriodicIO {
    double rpm = 0.0;
    double speed_diff = 0.0;

    LaserCan.Measurement measurement;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    mPeriodicIO.measurement = mLaserCAN.getMeasurement();

  }

  @Override
  public void writePeriodicOutputs() {
    // mLeftMotor.setVelocity(mPeriodicIO.rpm - mPeriodicIO.speed_diff);
    // mRightMotor.setVelocity(mPeriodicIO.rpm);
    mLeftMotor.setPercent(mPeriodicIO.rpm - mPeriodicIO.speed_diff);
    mRightMotor.setPercent(-mPeriodicIO.rpm);
  }

  @Override
  public void stop() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
  }

  @Override
  public void outputTelemetry() {
    putNumber("RPM/target", mPeriodicIO.rpm);

    putNumber("RPM/Left/Position", mLeftMotor.getPosition());
    putNumber("RPM/Right/Position", mRightMotor.getPosition());

    putNumber("RPM/Left/Velocity", mLeftMotor.getVelocity());
    putNumber("RPM/Right/Velocity", mRightMotor.getVelocity());

    LaserCan.Measurement measurement = mPeriodicIO.measurement;
    if (measurement != null) {
      putNumber("Laser/distance", measurement.distance_mm);
      putNumber("Laser/ambient", measurement.ambient);
      putNumber("Laser/budget_ms", measurement.budget_ms);
      putNumber("Laser/status", measurement.status);
    }
  }

  @Override
  public void reset() {
    stopCoral();
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void setSpeed(double rpm) {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = rpm;
  }

  public void intake() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kIntakeSpeed;
  }

  public void reverse() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kReverseSpeed;
  }

  public void index() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kIndexSpeed;
  }

  public void scoreL1() {
    mPeriodicIO.speed_diff = Constants.Coral.kSpeedDifference;
    mPeriodicIO.rpm = Constants.Coral.kL1Speed;
  }

  public void scoreL24() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kL24Speed;
  }

  public void stopCoral() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}