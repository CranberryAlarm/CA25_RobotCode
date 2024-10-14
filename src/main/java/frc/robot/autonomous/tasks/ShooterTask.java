package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Shooter;

public class ShooterTask extends Task {
  private Shooter m_shooter;
  private double m_speed;

  public ShooterTask(double speed) {
    m_shooter = Shooter.getInstance();
    m_speed = speed;
  }

  @Override
  public void start() {
    m_shooter.setSpeed(m_speed);
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
