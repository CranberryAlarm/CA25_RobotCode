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

  public boolean getWantsSpeedMode() {
    return this.getFilteredAxis(2) > k_triggerActivationThreshold;
  }

  // public boolean getWantsSlowMode() {
  // return this.getFilteredAxis(3) > k_triggerActivationThreshold;
  // }

  public boolean getWantsStow() {
    return this.getRawButton(3);
  }

  public boolean getWantsL2() {
    return this.getRawButton(1);
  }

  public boolean getWantsL3() {
    return this.getRawButton(2);
  }

  public boolean getWantsL4() {
    return this.getRawButton(4);
  }

  public boolean getWantsScoreCoral() {
    return this.getFilteredAxis(3) > k_triggerActivationThreshold;
  }

  public boolean getWantsIntakeCoral() {
    return this.getFilteredAxis(2) > k_triggerActivationThreshold;
  }

  public boolean getWantsA1() {
    return this.getHatDown();
  }

  public boolean getWantsA2() {
    return this.getHatUp();
  }

  public boolean getWantsStopAlgae() {
    return this.getHatRight();
  }

  public boolean getWantsElevatorReset() {
    return this.getRawButton(7);
  }

  public boolean getWantsEjectAlgae() {
    return this.getRawButton(6);
  }

  public boolean getWantsGroundAlgae() {
    return this.getRawButton(5);
  }

  // public boolean getWantsAlgaeStow() {
  // return this.getRawButton(1);
  // }

  // public boolean getWantsAlgaeGrab() {
  // return this.getRawButton(3);
  // }

  // public boolean getWantsAlgaeScore() {
  // return this.getRawButton(2);
  // }

  // public boolean getWantsAlgaeGroundIntake() {
  // return this.getRawButton(4);
  // }

  public void outputTelemetry() {
    SmartDashboard.putNumber(m_smartDashboardKey + "Forward", getForwardAxis());
    SmartDashboard.putNumber(m_smartDashboardKey + "Turn", getTurnAxis());
  }
}
