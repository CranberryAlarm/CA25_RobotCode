package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;

public class PPTestMode extends AutoModeBase {
    public void queueTasks() {
        // queueTask(new DriveForwardTask(1, 0.4));
        // queueTask(new DriveTrajectoryTask("TestPath", 9999, 9999));
        // queueTask(new DriveTrajectoryTask("SubLeftNote1", 9999, 9999));
        queueTask(new DriveTrajectoryTask("Note2"));
        queueTask(new DriveTrajectoryTask("Note2Back"));

        queueTask(new BrakeTask());
    }

    // public void queueTasks() {
    // // Spin up shooter
    // queueTask(new ParallelTask(
    // new IntakeTask(IntakeState.NONE, PivotTarget.STOW),
    // new ShooterTask(3000),
    // new WaitTask(2.0)));

    // // Shoot #1
    // queueTask(new ParallelTask(
    // new IntakeTask(IntakeState.EJECT, PivotTarget.STOW),
    // new WaitTask(1.0)));

    // // Intake to ground
    // queueTask(new ParallelTask(
    // new IntakeTask(IntakeState.INTAKE, PivotTarget.GROUND),
    // new WaitTask(0.1)));

    // // Drive to #2
    // queueTask(new DriveForwardTask(1, 0.4));

    // // Set intake to pulse
    // queueTask(new ParallelTask(
    // new IntakeTask(IntakeState.PULSE, PivotTarget.STOW),
    // new WaitTask(0.1)));

    // // Drive to sub
    // queueTask(new DriveForwardTask(1, -0.4));

    // // Shoot #2
    // queueTask(new ParallelTask(
    // new IntakeTask(IntakeState.EJECT, PivotTarget.STOW),
    // new WaitTask(2.0)));

    // // Drive to #3 and intake
    // queueTask(new ParallelTask(
    // new DriveTrajectoryTask("SubLeftNote1", 2, 1),
    // new IntakeTask(IntakeState.INTAKE, PivotTarget.GROUND),
    // new WaitTask(0.1)));

    // // Drive to sub and stow
    // queueTask(new ParallelTask(
    // new DriveTrajectoryTask("SubLeftNote2", 2, 1),
    // new IntakeTask(IntakeState.PULSE, PivotTarget.STOW),
    // new WaitTask(0.1)));

    // // Shoot #3
    // queueTask(new ParallelTask(
    // new IntakeTask(IntakeState.EJECT, PivotTarget.STOW),
    // new WaitTask(1.0)));

    // // DONE
    // queueTask(new ParallelTask(
    // new IntakeTask(IntakeState.NONE, PivotTarget.STOW),
    // new WaitTask(2.0),
    // new ShooterTask(0)));

    // queueTask(new BrakeTask());
    // }
}
