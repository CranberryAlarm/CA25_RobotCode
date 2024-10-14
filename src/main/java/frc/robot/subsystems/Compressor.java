package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Compressor extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  // The instantiation of this object will cause the compressor to start running
  private edu.wpi.first.wpilibj.Compressor m_compressor;
  private static Compressor m_instance;

  public static Compressor getInstance() {
    if (m_instance == null) {
      m_instance = new Compressor();
    }
    return m_instance;
  }

  private Compressor() {
    super("Compressor");

    m_compressor = new edu.wpi.first.wpilibj.Compressor(PneumaticsModuleType.CTREPCM);
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void stop() {
  }

  @Override
  public void outputTelemetry() {
    putNumber("Pressure", m_compressor.getPressure());
  }

  @Override
  public void reset() {
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  /*---------------------------------- Custom Private Functions ---------------------------------*/

}
