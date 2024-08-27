package frc.robot.subsystems.shooter.shooterSub;

public interface shooterMotorIO {

  public class OutputShooter {

    double volts;
    double amps;

    double speed;
  }

  public default OutputShooter getOutput() {
    return new OutputShooter();
  }

  public default void setVelocity(double topRPM, double bottomRPM) {}
}
