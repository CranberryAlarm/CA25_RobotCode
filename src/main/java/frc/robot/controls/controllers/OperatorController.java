package frc.robot.controls.controllers;

public class OperatorController extends FilteredController {

  public OperatorController(int port) {
    super(port, false, false);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  // Axis
  private final double k_triggerActivationThreshold = 0.5;

  public double getElevatorAxis() {
    return -this.getFilteredAxis(1);
  }

  // public boolean getWantsTriggerSomething2() {
  // return this.getFilteredAxis(2) > k_triggerActivationThreshold;
  // }

  // CORAL
  public boolean getWantsCoralIntake() {
    return this.getRawButton(4);
  }

  public boolean getWantsCoralReverse() {
    return this.getRawButton(1);
  }

  public boolean getWantsCoralIndex() {
    return this.getRawButton(8);
  }

  public boolean getWantsCoralL1() {
    return this.getRawButton(3);
  }

  public boolean getWantsCoralL24() {
    return this.getRawButton(2);
  }

  // ELEVATOR
  public boolean getWantsElevatorReset() {
    return this.getRawButton(7);
  }

  public boolean getWantsElevatorStow() {
    return this.getHatDown();
  }

  public boolean getWantsElevatorL2() {
    return this.getHatLeft();
  }

  public boolean getWantsElevatorL3() {
    return this.getHatRight();
  }

  public boolean getWantsElevatorL4() {
    return this.getHatUp();
  }
}
