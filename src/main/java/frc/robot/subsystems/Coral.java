package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.thethriftybot.ThriftyNova.PIDConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.simulation.SimulatableCANSparkMax;
import frc.robot.subsystems.leds.LEDs;

public class Coral extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Coral mInstance;
  private PeriodicIO mPeriodicIO;
  public final LEDs m_leds = LEDs.getInstance();

  public static Coral getInstance() {
    if (mInstance == null) {
      mInstance = new Coral();
    }
    return mInstance;
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    REVERSE,
    INDEX,
    READY,
    SCORE
  }

  // private ThriftyNova mLeftMotor;
  // private ThriftyNova mRightMotor;
  private SimulatableCANSparkMax mLeftMotor;
  private SparkPIDController mLeftPIDController;
  private SimulatableCANSparkMax mRightMotor;
  private SparkPIDController mRightPIDController;

  private PIDConfig mPIDConfig;

  private LaserCan mLaserCAN;

  private Coral() {
    super("Coral");

    mPeriodicIO = new PeriodicIO();

    // mLeftMotor = new ThriftyNova(Constants.Coral.kLeftMotorId);
    // mRightMotor = new ThriftyNova(Constants.Coral.kRightMotorId);
    mLeftMotor = new SimulatableCANSparkMax(Constants.Coral.kLeftMotorId, MotorType.kBrushless);
    mRightMotor = new SimulatableCANSparkMax(Constants.Coral.kRightMotorId, MotorType.kBrushless);

    // mLeftMotor.factoryReset();
    // mRightMotor.factoryReset();

    // mLeftMotor.setBrakeMode(true);
    // mRightMotor.setBrakeMode(true);
    mLeftMotor.setIdleMode(IdleMode.kBrake);
    mRightMotor.setIdleMode(IdleMode.kBrake);

    // mLeftPIDController = mLeftMotor.getPIDController();
    // mLeftPIDController.setP(Constants.Coral.kP);
    // mLeftPIDController.setI(Constants.Coral.kI);
    // mLeftPIDController.setD(Constants.Coral.kD);
    // mLeftPIDController.setIZone(Constants.Coral.kIZone);

    // mRightPIDController = mRightMotor.getPIDController();
    // mRightPIDController.setP(Constants.Coral.kP);
    // mRightPIDController.setI(Constants.Coral.kI);
    // mRightPIDController.setD(Constants.Coral.kD);
    // mRightPIDController.setIZone(Constants.Coral.kIZone);

    // mLeftMotor.setMaxCurrent(CurrentType.STATOR, Constants.Coral.kMaxCurrent);
    // mRightMotor.setMaxCurrent(CurrentType.STATOR, Constants.Coral.kMaxCurrent);

    // mLeftMotor.setMaxCurrent(CurrentType.STATOR, 200.0);
    // mRightMotor.setMaxCurrent(CurrentType.STATOR, 200.0);

    // mLeftMotor.setMaxCurrent(CurrentType.SUPPLY, 200.0);
    // mRightMotor.setMaxCurrent(CurrentType.SUPPLY, 200.0);

    // mLeftMotor.setRampUp(0.5);
    // mRightMotor.setRampUp(0.5);

    // mLeftMotor.setMaxOutput(1.0);
    // mRightMotor.setMaxOutput(1.0);

    // mPIDConfig = mRightMotor.pid0;
    // mPIDConfig.setP(Constants.Coral.kP);
    // mPIDConfig.setI(Constants.Coral.kI);
    // mPIDConfig.setD(Constants.Coral.kD);
    // mPIDConfig.setIZone(Constants.Coral.kIZone);

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

    int index_debounce = 0;

    LaserCan.Measurement measurement;

    IntakeState state = IntakeState.NONE;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    mPeriodicIO.measurement = mLaserCAN.getMeasurement();

    checkAutoTasks();
  }

  @Override
  public void writePeriodicOutputs() {
    // mLeftMotor.setVelocity(mPeriodicIO.rpm - mPeriodicIO.speed_diff);
    // mRightMotor.setVelocity(mPeriodicIO.rpm);

    // mLeftMotor.setPercent(mPeriodicIO.rpm - mPeriodicIO.speed_diff);
    // mRightMotor.setPercent(-mPeriodicIO.rpm);

    mLeftMotor.set(mPeriodicIO.rpm - mPeriodicIO.speed_diff);
    mRightMotor.set(-mPeriodicIO.rpm);
  }

  @Override
  public void stop() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.state = IntakeState.NONE;
  }

  @Override
  public void outputTelemetry() {
    putNumber("RPM/target", mPeriodicIO.rpm);

    // putNumber("RPM/Left/Position", mLeftMotor.getPosition());
    // putNumber("RPM/Right/Position", mRightMotor.getPosition());

    // putNumber("RPM/Left/Velocity", mLeftMotor.getVelocity());
    // putNumber("RPM/Right/Velocity", mRightMotor.getVelocity());

    LaserCan.Measurement measurement = mPeriodicIO.measurement;
    if (measurement != null) {
      putNumber("Laser/distance", measurement.distance_mm);
      putNumber("Laser/ambient", measurement.ambient);
      putNumber("Laser/budget_ms", measurement.budget_ms);
      putNumber("Laser/status", measurement.status);

      putBoolean("Laser/hasCoral", isHoldingCoralViaLaserCAN());
    }
  }

  @Override
  public void reset() {
    stopCoral();
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public boolean isHoldingCoralViaLaserCAN() {
    return mPeriodicIO.measurement.distance_mm < 75.0;
  }

  public void setSpeed(double rpm) {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = rpm;
  }

  public void intake() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kIntakeSpeed;
    mPeriodicIO.state = IntakeState.INTAKE;

    m_leds.setColor(Color.kYellow);
  }

  public void reverse() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kReverseSpeed;
    mPeriodicIO.state = IntakeState.REVERSE;
  }

  public void index() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kIndexSpeed;
    mPeriodicIO.state = IntakeState.INDEX;

    m_leds.setColor(Color.kBlue);
  }

  public void scoreL1() {
    mPeriodicIO.speed_diff = Constants.Coral.kSpeedDifference;
    mPeriodicIO.rpm = Constants.Coral.kL1Speed;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  public void scoreL24() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kL24Speed;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  public void stopCoral() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.state = IntakeState.NONE;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  private void checkAutoTasks() {
    switch (mPeriodicIO.state) {
      case INTAKE:
        if (isHoldingCoralViaLaserCAN()) {
          mPeriodicIO.index_debounce++;

          if (mPeriodicIO.index_debounce > 10) {
            mPeriodicIO.index_debounce = 0;
            index();
          }
        }
        break;
      case INDEX:
        if (!isHoldingCoralViaLaserCAN()) {
          stopCoral();

          mPeriodicIO.state = IntakeState.READY;
          m_leds.setColor(Color.kBlue);
        }
        break;
      default:
        break;
    }
  }
}
