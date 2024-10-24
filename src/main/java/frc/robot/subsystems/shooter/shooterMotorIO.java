package frc.robot.subsystems.shooter;

public interface shooterMotorIO {

  public class OutputShooter {

    double volts;
    double amps;
    double acceleration;
    double speed;
    double position;
  }

  public default OutputShooter getOutput() {
    return new OutputShooter();
  }

  public default void setVelocity(double topRPM, double bottomRPM) {}

  public default void setVolts(double volts){

  }
}
