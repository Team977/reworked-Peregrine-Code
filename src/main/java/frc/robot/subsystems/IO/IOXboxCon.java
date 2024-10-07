package frc.robot.subsystems.IO;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class IOXboxCon implements IOMoudlue {

  private static final CommandXboxController driverCon = new CommandXboxController(0);
  private static final CommandXboxController opporatorCon = new CommandXboxController(1);

  public DoubleSupplier getXPower() {
    return () -> -driverCon.getLeftY();
  }

  public DoubleSupplier getYPower() {
    return () -> -driverCon.getLeftX();
  }

  public DoubleSupplier getOmegaPower() {
    return () -> -driverCon.getRightX();
  }

  public Trigger getShooterReady() {
    return opporatorCon.rightTrigger();
  }

  public Trigger getShoot() {
    return driverCon.rightTrigger();
  }

  public Trigger ReverseIntake() {
    return opporatorCon.b();
  }

  public Trigger Intake() {
    return opporatorCon.a();
  }

  public Trigger setModeSpeeker() {
    return opporatorCon.y();
  }

  public Trigger setModeAmp() {
    return opporatorCon.x();
  }

  public Trigger setModeNote() {
    return opporatorCon.rightBumper();
  }

  public Trigger setModeFeed() {
    return opporatorCon.leftBumper();
  }

  public Trigger setAutoRotateOn() {
    return opporatorCon.back();
  }

  public Trigger setAutoRotateOff() {
    return opporatorCon.start();
  }

  public Trigger setPassiveSwitchOn() {
    return opporatorCon.leftStick();
  }

  public Trigger setPassiveSwitchOff() {
    return opporatorCon.rightBumper();
  }

  public Trigger setDriveModeFast() {
    return new Trigger(() -> false);
  }

  public Trigger setDriveModeNormal() {
    return new Trigger(() -> false);
  }

  public Trigger setDriveModeSlow() {
    return new Trigger(() -> false);
  }

  public Trigger setGoalMannule(){
    return opporatorCon.leftStick();
  }

    
  public default Trigger resetPose(){
    return opporatorCon.rightStick();
}
}
