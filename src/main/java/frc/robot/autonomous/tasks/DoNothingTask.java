package frc.robot.autonomous.tasks;

public class DoNothingTask extends Task {
  @Override
  public void start() {
    System.out.println("Starting do nothing auto...");
  }

  @Override
  public void update() {
    System.out.println("Do nothing auto complete");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
