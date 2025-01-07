package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static class Robot {
    public static final double k_width = 26; // Inches
    public static final double k_length = 28; // Inches
  }

  public static class Elevator {
    public static final int kElevatorLeftMotorId = 9;
    public static final int kElevatorRightMotorId = 10;

    public static final double kP = 0.030;
    public static final double kI = 0.00005;
    public static final double kD = 0.0;
    public static final double kIZone = 5.0;
    public static final double kG = 0.5;

    public static final int kMaxCurrent = 40;
    public static final double kMaxPowerUp = 0.1;
    public static final double kMaxPowerDown = 0.1;

    public static final double kStowHeight = 0.0;
    public static final double kL2Height = 9.0;
    public static final double kL3Height = 25.14;
    public static final double kL4Height = 51.0;
    public static final double kMaxHeight = 56.2;
    public static final double kGroundAlgaeHeight = 0.0;
    public static final double kScoreAlgaeHeight = 0.0;
    public static final double kLowAlgaeHeight = 0.0;
    public static final double kHighAlgaeHeight = 0.0;
  }

  public static class Coral {
    public static final int kLeftMotorId = 11;
    public static final int kRightMotorId = 12;

    public static final int kLaserId = 0;
    public static final int kColorId = 16;

    public static final double kMaxCurrent = 20;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kIZone = 0;

    public static final double kIntakeSpeed = 0.3;
    public static final double kReverseSpeed = -0.3;
    public static final double kL1Speed = 0.4;
    public static final double kL24Speed = 0.4;
    public static final double kIndexSpeed = 0.2;
    public static final double kSpeedDifference = kL1Speed * 0.5;
  }

  public static class Algae {
    public static final int kWristMotorId = 13;

    public static final int kIntakeMotorId = 14;
  }

  public static class Intake {
    // Motors
    public static final int kIntakeMotorId = 9;
    public static final int kPivotMotorId = 10;

    // DIO
    public static final int k_pivotEncoderId = 0;
    public static final int k_intakeLimitSwitchId = 2;

    // Absolute encoder offset
    public static final double k_pivotEncoderOffset = 0.166842; // Straight up, sketchy to reset to "up"

    // Pivot set point angles
    public static final double k_pivotAngleGround = 60;
    public static final double k_pivotAngleSource = 190;
    public static final double k_pivotAngleAmp = k_pivotAngleSource;
    public static final double k_pivotAngleStow = 275;

    // Intake speeds
    public static final double k_intakeSpeed = 0.7;
    public static final double k_ejectSpeed = -0.45;
    public static final double k_feedShooterSpeed = -0.5;
  }

  // PCM
  public static final int kPCMId = 0;
  public static final int kIntakeSolenoidForwardId = 2;

  // DIO

  // Shooter
  public static final int kShooterLeftMotorId = 12;
  public static final int kShooterRightMotorId = 13;

  public static final double kShooterP = 0.00005;
  public static final double kShooterI = 0.0;
  public static final double kShooterD = 0.0;
  public static final double kShooterFF = 0.0002;

  public static final double kShooterMinOutput = 0;
  public static final double kShooterMaxOutput = 1;

  // Climber
  public static final int kClimberLeftMotorId = 14;
  public static final int kClimberRightMotorId = 15;
  public static final double kClimberClimbSpeed = 600.0; // RPM
  public static final double kClimberReleaseSpeed = -600.0; // RPM

  public static final double kClimberGearRatio = 1.0 / 12.0;

  public static final double kClimberP = 0.001;
  public static final double kClimberI = 0.0;
  public static final double kClimberD = 0.0;
  public static final double kClimberMinOutput = -0.5;

  public static final double kClimberMaxOutput = 0.5;

  // Drivetrain
  public static class Drive {
    public static final double kP = 0.00085;// 3.6954;//0.085;
    public static final double kI = 0.0;// 0.0;
    public static final double kD = 0.0;// 0.0;

    public static final double kS = 0.1695;// 0.01;
    public static final double kV = 2.8559;// 2.6;
    public static final double kA = 0.4864;

    public static final int kFLMotorId = 8;
    public static final int kBLMotorId = 7;
    public static final int kFRMotorId = 6;
    public static final int kBRMotorId = 5;
  }

  public static class Field {
    public static final double k_width = Units.feetToMeters(54.0);
    public static final double k_length = Units.feetToMeters(27.0);

    // TODO: Add left and right subwoofer starting poses
    public static final Pose2d redCenterPose2d = new Pose2d(15.19, 5.50, new Rotation2d(Units.degreesToRadians(180.0)));
    public static final Pose2d blueCenterPose2d = new Pose2d(1.27, 5.50, new Rotation2d(0));
  }

  public static class LEDs {
    public static final int k_PWMId = 0;
    public static final int k_totalLength = 300;
  }
}
