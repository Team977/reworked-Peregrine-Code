package frc.robot.subsystems.IO;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IOJoystick implements IOMoudlue {
    
    private static final CommandJoystick driverCon = new CommandJoystick(0);
    private static final CommandXboxController opporatorCon = new CommandXboxController(1);

     public default DoubleSupplier getXPower() {
    return () -> driverCon.getX();
  }

  public default DoubleSupplier getYPower() {
    return () -> driverCon.getY();
  }

  public default DoubleSupplier getOmegaPower() {
    return () -> driverCon.getZ();
  }

  public default Trigger getShooterReady() {
    return opporatorCon.leftTrigger();
  }

  public default Trigger getShoot() {
    return driverCon.button(0);
  }

  public default Trigger ReverseIntake() {
    return opporatorCon.b();
  }

  public default Trigger Intake() {
    return opporatorCon.a();
  }

  public default Trigger setModeSpeeker() {
    return opporatorCon.y();
  }

  public default Trigger setModeAmp() {
    return opporatorCon.x();

  public default Trigger setModeNote() {
    return opporatorCon.rightBumper();
  }

  public default Trigger setModeFeed() {
    return opporatorCon.leftBumper();
  }

  public default Trigger setAutoRotateOn() {
    return opporatorCon.back();
  }

  public default Trigger setAutoRotateOff() {
    return opporatorCon.start();
  }

  public default Trigger setPassiveSwitchOn() {
    return opporatorCon.leftStick();
  }

  public default Trigger setPassiveSwitchOff() {
    return opporatorCon.rightBumper();
  }

  public default Trigger setDriveModeFast() {
    return driverCon.button(1);
  }

  public default Trigger setDriveModeNormal() {
    return driverCon.button(2);
  }

  public default Trigger setDriveModeSlow() {
    return driverCon.button(3);
  }

}
