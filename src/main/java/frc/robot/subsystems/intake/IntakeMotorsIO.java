package frc.robot.subsystems.intake;

public interface IntakeMotorsIO {

  public default void setSpeed(double speed) {}

  public default OutputIntake getOutputs() {
    return new OutputIntake();
  }

  public default boolean HasNote() {
    return false;
  }
  ;

  public class OutputIntake {

    double speed;
  }
}
