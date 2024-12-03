package frc.robot.autonomous.tasks;

import java.nio.file.Path;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.controllers.PPRamseteController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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
  private PPRamseteController m_driveController;

  public DriveTrajectoryTask(String pathName) {
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
        m_drive.getPose().getRotation());

    if (m_autoPath.isReversed()) {
      DriverStation.reportWarning("===== PATH IS REVERSED =====", false);
    }


    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
    m_driveController = new PPRamseteController(2.0, 0.7);
  }

  @Override
  public void start() {
    m_runningTimer.reset();
    m_runningTimer.start();

    // Set the initial Pose2d
    if (!m_drive.poseWasSet()) {
      m_drive.setPose(m_autoPath.getStartingDifferentialPose());
      DriverStation.reportWarning(m_autoPath.getStartingDifferentialPose().toString(), false);
    }

    // DEBUG Trajectory //////////////////////////////////////////////
    Trajectory adjustedTrajectory = TrajectoryGenerator.generateTrajectory(
        m_autoPath.getPathPoses(),
        new TrajectoryConfig(
            m_autoPath.getGlobalConstraints().getMaxVelocityMps(),
            m_autoPath.getGlobalConstraints().getMaxAccelerationMpsSq()));
    Logger.recordOutput("Auto/DriveTrajectory/TargetTrajectory", adjustedTrajectory);
    /////////////////////////////////////////////////////////////////

    m_drive.clearTurnPIDAccumulation();
    DriverStation.reportWarning("Running path for " + DriverStation.getAlliance().toString(), false);
  }

  @Override
  public void update() {
    State goal = m_autoTrajectory.sample(m_runningTimer.get());
    if (m_autoPath.isReversed()) {
      goal = goal.reverse();
    }
    ChassisSpeeds chassisSpeeds = m_driveController.calculateRobotRelativeSpeeds(m_drive.getPose(), goal);

    m_drive.drive(chassisSpeeds);

    m_isFinished |= m_runningTimer.get() >= m_autoTrajectory.getTotalTimeSeconds();

    SmartDashboard.putNumber(m_smartDashboardKey + "vx", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber(m_smartDashboardKey + "vy", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(m_smartDashboardKey + "vr", chassisSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void updateSim() {
    if (!RobotBase.isReal()) {
      PathPlannerTrajectory.State autoState = (PathPlannerTrajectory.State) m_autoTrajectory
          .sample(m_runningTimer.get());

      Pose2d targetPose2d = new Pose2d(
          autoState.positionMeters.getX(),
          autoState.positionMeters.getY(),
          autoState.heading);

      m_drive.setPose(targetPose2d);
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
