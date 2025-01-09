package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.simulation.SimulatableCANSparkMax;

public class Elevator extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Elevator mInstance;
  private PeriodicIO mPeriodicIO;

  // private static final double kPivotCLRampRate = 0.5;
  // private static final double kCLRampRate = 0.5;

  public static Elevator getInstance() {
    if (mInstance == null) {
      mInstance = new Elevator();
    }
    return mInstance;
  }

  private SimulatableCANSparkMax mLeftMotor;
  private RelativeEncoder mLeftEncoder;
  private SparkPIDController mLeftPIDController;

  private SimulatableCANSparkMax mRightMotor;
  private RelativeEncoder mRightEncoder;
  private SparkPIDController mRightPIDController;

  private TrapezoidProfile mProfile;
  private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
  private double prevUpdateTime = Timer.getFPGATimestamp();

  // private RelativeEncoder mLeftEncoder;
  // private SparkMaxLimitSwitch mLowerLimit;
  // private SparkMaxLimitSwitch mUpperLimit;

  // private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);

  private void setUpElevatorMotor(SimulatableCANSparkMax motor, SparkPIDController pidController) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(Constants.Elevator.kMaxCurrent);
    // mLeftEncoder = motor.getEncoder();

    pidController.setP(Constants.Elevator.kP);
    pidController.setI(Constants.Elevator.kI);
    pidController.setD(Constants.Elevator.kD);
    pidController.setIZone(Constants.Elevator.kIZone);
    // pidController.setIMaxAccum(0.001, 0)

    // mLeftPIDController.setOutputRange(Constants.Elevator.kMaxPowerUp,
    // Constants.Elevator.kMaxPowerDown);

    // motor.setClosedLoopRampRate(kExtensionCLRampRate);

    // mLowerLimit = mLeftMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    // mUpperLimit = mLeftMotor.getForwardLimitSwitch(Type.kNormallyOpen);
  }

  private Elevator() {
    super("Elevator");

    mPeriodicIO = new PeriodicIO();

    // LEFT ELEVATOR MOTOR
    mLeftMotor = new SimulatableCANSparkMax(Constants.Elevator.kElevatorLeftMotorId, MotorType.kBrushless);
    mLeftEncoder = mLeftMotor.getEncoder();
    mLeftPIDController = mLeftMotor.getPIDController();
    setUpElevatorMotor(mLeftMotor, mLeftPIDController);

    // RIGHT ELEVATOR MOTOR
    mRightMotor = new SimulatableCANSparkMax(Constants.Elevator.kElevatorRightMotorId, MotorType.kBrushless);
    mRightEncoder = mRightMotor.getEncoder();
    mRightPIDController = mRightMotor.getPIDController();
    setUpElevatorMotor(mRightMotor, mRightPIDController);

    // mRightMotor.setInverted(true);
    mRightMotor.follow(mLeftMotor, true);

    mLeftMotor.burnFlash();
    mRightMotor.burnFlash();

    mProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAcceleration));
  }

  public enum ElevatorState {
    NONE,
    STOW,
    L2,
    L3,
    L4,
    A1,
    A2
  }

  private static class PeriodicIO {
    double elevator_target = 0.0;
    double elevator_power = 0.0;

    boolean is_elevator_pos_control = false;

    ElevatorState state = ElevatorState.STOW;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    // TODO: Use this pattern to only drive slowly when we're really high up
    // if(mPivotEncoder.getPosition() > Constants.kPivotScoreCount) {
    // mPeriodicIO.is_pivot_low = true;
    // } else {
    // mPeriodicIO.is_pivot_low = false;
    // }
  }

  @Override
  public void writePeriodicOutputs() {
    double curTime = Timer.getFPGATimestamp();
    double dt = curTime - prevUpdateTime;
    prevUpdateTime = curTime;
    if (mPeriodicIO.is_elevator_pos_control) {
      // Update goal
      mGoalState.position = mPeriodicIO.elevator_target;

      // Calculate new state
      prevUpdateTime = curTime;
      mCurState = mProfile.calculate(dt, mCurState, mGoalState);

      // Set PID controller to new state
      mLeftPIDController.setReference(
          mCurState.position,
          CANSparkMax.ControlType.kPosition,
          0,
          Constants.Elevator.kG,
          ArbFFUnits.kVoltage);
    } else {
      mCurState.position = mLeftEncoder.getPosition();
      mCurState.velocity = 0;
      mLeftMotor.set(mPeriodicIO.elevator_power);
    }
  }

  @Override
  public void stop() {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = 0.0;

    mLeftMotor.set(0.0);
  }

  @Override
  public void outputTelemetry() {
    putNumber("Position/Current", mLeftEncoder.getPosition());
    putNumber("Position/Target", mPeriodicIO.elevator_target);
    putNumber("Velocity/Current", mLeftEncoder.getVelocity());

    putNumber("Position/Setpoint", mCurState.position);
    putNumber("Velocity/Setpoint", mCurState.velocity);

    putNumber("Current/Left", mLeftMotor.getOutputCurrent());
    putNumber("Current/Right", mRightMotor.getOutputCurrent());

    putNumber("Output/Left", mLeftMotor.getAppliedOutput());
    putNumber("Output/Right", mRightMotor.getAppliedOutput());

    putNumber("State", mPeriodicIO.state);
  }

  @Override
  public void reset() {
    mLeftEncoder.setPosition(0.0);
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public ElevatorState getState() {
    return mPeriodicIO.state;
  }

  public void setElevatorPower(double power) {
    putNumber("setElevatorPower", power);
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = power;
  }

  public void goToElevatorStow() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kStowHeight;
    mPeriodicIO.state = ElevatorState.STOW;
  }

  public void goToElevatorL2() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kL2Height;
    mPeriodicIO.state = ElevatorState.L2;
  }

  public void goToElevatorL3() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kL3Height;
    mPeriodicIO.state = ElevatorState.L3;
  }

  public void goToElevatorL4() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kL4Height;
    mPeriodicIO.state = ElevatorState.L4;
  }

  public void goToAlgaeLow() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kLowAlgaeHeight;
    mPeriodicIO.state = ElevatorState.A1;
  }

  public void goToAlgaeHigh() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kHighAlgaeHeight;
    mPeriodicIO.state = ElevatorState.A2;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
