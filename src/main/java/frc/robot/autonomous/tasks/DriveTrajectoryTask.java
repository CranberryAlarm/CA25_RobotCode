package frc.robot.autonomous.tasks;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.nio.file.Path;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class DriveTrajectoryTask extends Task {
  private Drivetrain m_drive;
  private PathPlannerTrajectory m_autoTrajectory;
  private boolean m_isFinished = false;
  private String m_smartDashboardKey = "DriveTrajectoryTask/";
  private PathPlannerPath m_autoPath = null;

  private final Timer m_runningTimer = new Timer();
  private PPLTVController m_driveController;

  public DriveTrajectoryTask(String pathName, double maxSpeed, double maxAcceleration) {
    m_drive = Drivetrain.getInstance();
    Path trajectoryPath = null;

    try {
      if (RobotBase.isReal()) {
        System.out.println("Running on the robot!");
        trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + pathName);
      } else {
        System.out.println("Running in simulation!");
        trajectoryPath = Filesystem.getLaunchDirectory().toPath().resolve("PathWeaver/output/" + pathName);
      }

      System.out.println("Loading path from:\n" + trajectoryPath.toString());
      m_autoPath = PathPlannerPath.fromPathFile(pathName);
      // System.out.println(m_autoPath.numPoints());
    } catch (Exception ex) {
      DriverStation.reportError("Unable to load PathPlanner trajectory: " + pathName, ex.getStackTrace());
      m_isFinished = true;
    }

    m_autoTrajectory = new PathPlannerTrajectory(
        m_autoPath,
        new ChassisSpeeds(),
        m_drive.getPose().getRotation(),
        m_drive.getRobotConfig()
        );

    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
    // TODO: Tune these values
    m_driveController = new PPLTVController(0.1);
  }

  @Override
  public void start() {
    m_runningTimer.reset();
    m_runningTimer.start();

    // Set the initial Pose2d
    m_drive.setPose(m_autoPath.getStartingDifferentialPose());
    DriverStation.reportWarning(m_autoPath.getStartingDifferentialPose().toString(), false);

    m_drive.clearTurnPIDAccumulation();
    DriverStation.reportWarning("Running path for " + DriverStation.getAlliance().toString(), false);
  }

  @Override
  public void update() {
    PathPlannerTrajectoryState goal = m_autoTrajectory.sample(m_runningTimer.get());
    ChassisSpeeds chassisSpeeds = m_driveController.calculateRobotRelativeSpeeds(m_drive.getPose(), goal);

    m_drive.drive(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond);

    m_isFinished |= m_runningTimer.get() >= m_autoTrajectory.getTotalTimeSeconds();

    SmartDashboard.putNumber(m_smartDashboardKey + "vx", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber(m_smartDashboardKey + "vy", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(m_smartDashboardKey + "vr", chassisSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void updateSim() {
    if (!RobotBase.isReal()) {
      PathPlannerTrajectoryState autoState = m_autoTrajectory
          .sample(m_runningTimer.get());

      m_drive.setPose(autoState.pose);
    }
  }

  public Pose2d getStartingPose() {
    return m_autoPath.getStartingDifferentialPose();
  }

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }

  @Override
  public void done() {
    DriverStation.reportWarning("Auto trajectory done", false);
    m_drive.drive(0, 0);
  }
}
