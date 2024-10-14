package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.PivotTarget;

public class IntakeTask extends Task {
  private Intake m_intake;
  private IntakeState m_intakeState;
  private PivotTarget m_pivotTarget;

  public IntakeTask(IntakeState intakeState, PivotTarget pivotTarget) {
    m_intake = Intake.getInstance();
    m_intakeState = intakeState;
    m_pivotTarget = pivotTarget;
  }

  @Override
  public void start() {
    m_intake.setState(m_intakeState);
    m_intake.setPivotTarget(m_pivotTarget);
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
