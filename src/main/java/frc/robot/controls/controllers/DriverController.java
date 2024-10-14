package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriverController extends FilteredController {
  private String m_smartDashboardKey = "DriverInput/";

  public DriverController(int port) {
    super(port, false, false);
  }

  public DriverController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  // Axis
  private final double k_triggerActivationThreshold = 0.5;

  public double getForwardAxis() {
    return -this.getFilteredAxis(1);
  }

  public double getTurnAxis() {
    return -this.getFilteredAxis(4);
  }

  public double getShooterAxis() {
    return (this.getFilteredAxis(3) - 0.5) * 2;
  }

  public boolean getWantsFullIntake() {
    return this.getRawButton(1);
  }

  public boolean getWantsIntake() {
    return this.getRawButton(6);
  }

  public boolean getWantsEject() {
    return this.getRawButton(2);
  }

  public boolean getWantsSource() {
    return this.getRawButton(3);
  }

  public boolean getWantsStow() {
    return this.getRawButton(4);
  }

  // public boolean getWantsFire() {
  // return this.getHatUpPressed();
  // }

  public boolean getWantsMoreSpeed() {
    return this.getHatUp();
  }

  public boolean getWantsLessSpeed() {
    return this.getHatDown();
  }

  public boolean getWantsShooterStop() {
    return this.getRawButton(5);
  }

  public boolean getWantsSpeedMode() {
    return this.getFilteredAxis(2) > k_triggerActivationThreshold;
  }

  public boolean getWantsSlowMode() {
    return this.getFilteredAxis(3) > k_triggerActivationThreshold;
  }

  // Buttons
  public boolean getWantsSomethingToggle() {
    return this.getRawButtonPressed(1);
  }

  public boolean getWantsSomething2Toggle() {
    return this.getRawButtonPressed(3);
  }

  public boolean getWantsResetGyro() {
    return this.getRawButton(4);
  }

  // public boolean getWantsBrake() {
  // return this.getRawButton(5);
  // }

  public void outputTelemetry() {
    SmartDashboard.putNumber(m_smartDashboardKey + "Forward", getForwardAxis());
    SmartDashboard.putNumber(m_smartDashboardKey + "Turn", getTurnAxis());
  }
}
