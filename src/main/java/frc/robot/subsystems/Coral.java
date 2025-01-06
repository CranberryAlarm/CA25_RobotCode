package frc.robot.subsystems;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;

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

  private Coral() {
    super("Coral");

    mPeriodicIO = new PeriodicIO();

    mLeftMotor = new ThriftyNova(Constants.Coral.kLeftMotorId);
    mRightMotor = new ThriftyNova(Constants.Coral.kRightMotorId);

    mLeftMotor.setBrakeMode(false);
    mRightMotor.setBrakeMode(false);

    mLeftMotor.setMaxCurrent(CurrentType.STATOR, 20);
    mRightMotor.setMaxCurrent(CurrentType.STATOR, 20);

    // TODO: PID?

    // TODO: Make sure we're inverting the correct motor
    mLeftMotor.setInversion(true);
  }

  private static class PeriodicIO {
    double shooter_rpm = 0.0;

    double left_rpm = 0.0;
    double right_rpm = 0.0;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {

  }

  @Override
  public void writePeriodicOutputs() {
    // System.out.println(mPeriodicIO.left_rpm);
    mLeftMotor.setVelocity(mPeriodicIO.left_rpm);
    mRightMotor.setVelocity(mPeriodicIO.right_rpm);
  }

  @Override
  public void stop() {
    stopCoral();
  }

  @Override
  public void outputTelemetry() {
    // putNumber("Speed (RPM):", mPeriodicIO.shooter_rpm);
    putNumber("Left speed:", mLeftMotor.getVelocity());
    putNumber("Right speed:", mRightMotor.getVelocity());
  }

  @Override
  public void reset() {
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void setSpeed(double rpm) {
    mPeriodicIO.left_rpm = rpm;
    mPeriodicIO.right_rpm = rpm;
  }

  public void stopCoral() {
    mPeriodicIO.left_rpm = 0.0;
    mPeriodicIO.right_rpm = 0.0;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
