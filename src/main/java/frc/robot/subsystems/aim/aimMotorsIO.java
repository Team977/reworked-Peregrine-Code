package frc.robot.subsystems.aim;

import edu.wpi.first.math.geometry.Rotation2d;

public interface aimMotorsIO {

  public class OutputAim {

    double Velocity;
    Rotation2d Rotation;

    double Current;
    double Volts;
  }

  public default boolean setAngle(Rotation2d angle) {
    return true;
  }
  ;

  public default void stop() {}

  public default void setVolts(double Volts) {}
  ;

  public default OutputAim getOutputs() {
    return new OutputAim();
  }
  ;

  public default void resetAimZeroPosition() {}
  ;
}
