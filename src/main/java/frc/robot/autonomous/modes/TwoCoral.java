package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.ScoreCoralTask;
import frc.robot.autonomous.tasks.WaitTask;

public class TwoCoral extends AutoModeBase {
    public void queueTasks() {
        queueTask(new DriveTrajectoryTask("Coral1"));
        // queueTask(new DriveTrajectoryTask("Coral2Intake"));
        // queueTask(new DriveTrajectoryTask("Coral2Score"));
        queueTask(new WaitTask(1));
        queueTask(new ScoreCoralTask());

        queueTask(new BrakeTask());
    }
}
