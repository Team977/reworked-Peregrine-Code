package frc.robot.subsystems.IO;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IOJoystick implements IOMoudlue {
    
    private static final CommandJoystick driverCon = new CommandJoystick(0);
    private static final CommandXboxController opporatorCon = new CommandXboxController(1);

     public  DoubleSupplier getXPower() {
    return () -> driverCon.getX();
  }

  public  DoubleSupplier getYPower() {
    return () -> driverCon.getY();
  }

  public  DoubleSupplier getOmegaPower() {
    return () -> driverCon.getZ();
  }

  public  Trigger getShooterReady() {
    return opporatorCon.leftTrigger();
  }

  public  Trigger getShoot() {
    return driverCon.button(0);
  }

  public  Trigger ReverseIntake() {
    return opporatorCon.b();
  }

  public  Trigger Intake() {
    return opporatorCon.a();
  }

  public  Trigger setModeSpeeker() {
    return opporatorCon.y();
  }

  public  Trigger setModeAmp() {
    return opporatorCon.x();}

  public  Trigger setModeNote() {
    return opporatorCon.rightBumper();
  }

  public  Trigger setModeFeed() {
    return opporatorCon.leftBumper();
  }

  public  Trigger setAutoRotateOn() {
    return opporatorCon.back();
  }

  public  Trigger setAutoRotateOff() {
    return opporatorCon.start();
  }

  public  Trigger setPassiveSwitchOn() {
    return opporatorCon.leftStick();
  }

  public  Trigger setPassiveSwitchOff() {
    return opporatorCon.rightBumper();
  }

  public  Trigger setDriveModeFast() {
    return driverCon.button(1);
  }

  public  Trigger setDriveModeNormal() {
    return driverCon.button(2);
  }

  public  Trigger setDriveModeSlow() {
    return driverCon.button(3);
  }

}
