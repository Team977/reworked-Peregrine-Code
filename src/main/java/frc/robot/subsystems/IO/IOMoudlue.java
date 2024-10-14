package frc.robot.subsystems.IO;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public interface IOMoudlue {

  public default DoubleSupplier getXPower() {
    return () -> 0;
  }

  public default DoubleSupplier getYPower() {
    return () -> 0;
  }

  public default DoubleSupplier getOmegaPower() {
    return () -> 0;
  }

  public default Trigger getShooterReady() {
    return new Trigger(() -> false);
  }

  public default Trigger getShoot() {
    return new Trigger(() -> false);
  }

  public default Trigger ReverseIntake() {
    return new Trigger(() -> false);
  }

  public default Trigger Intake() {
    return new Trigger(() -> false);
  }

  public default Trigger setModeSpeeker() {
    return new Trigger(() -> false);
  }

  public default Trigger setModeAmp() {
    return new Trigger(() -> false);
  }

  public default Trigger setModeNote() {
    return new Trigger(() -> false);
  }

  public default Trigger setModeFeed() {
    return new Trigger(() -> false);
  }

  public default Trigger setAutoRotateOn() {
    return new Trigger(() -> false);
  }

  public default Trigger setAutoRotateOff() {
    return new Trigger(() -> false);
  }

  public default Trigger setPassiveSwitchOn() {
    return new Trigger(() -> false);
  }

  public default Trigger setPassiveSwitchOff() {
    return new Trigger(() -> false);
  }

  public default Trigger setDriveModeFast() {
    return new Trigger(() -> false);
  }

  public default Trigger setDriveModeNormal() {
    return new Trigger(() -> false);
  }

  public default Trigger setDriveModeSlow() {
    return new Trigger(() -> false);
  }

  public default Trigger setGoalMannule() {
    return new Trigger(() -> false);
  }

  public default Trigger resetPose() {
    return new Trigger(() -> false);
  }
}
