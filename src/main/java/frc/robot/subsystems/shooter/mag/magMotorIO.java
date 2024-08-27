package frc.robot.subsystems.shooter.mag;

public interface magMotorIO {

  public class OutputMag {

    double volts;
    double amps;

    double speed;
  }

  public default OutputMag getOutput() {
    return new OutputMag();
  }

  public default void setSpeed(double speed) {}
  ;
}
