package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Coral;

public class ScoreCoralTask extends Task {
  private Coral m_coral;

  public ScoreCoralTask() {
    m_coral = Coral.getInstance();
  }

  @Override
  public void start() {
    m_coral.scoreL1();
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void done() {

  }
}
