package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class RobotTelemetry {
  public static void print(String output) {
    System.out.println(
        String.format("%.3f", Timer.getFPGATimestamp()) + " || " + output);
  }

  public static void print(Object output) {
    System.out.println(
        String.format("%.3f", Timer.getFPGATimestamp()) + " || " + output.toString());
  }
}
