package frc.robot.subsystems.IO;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class IOSim implements IOMoudlue {

  private final Joystick simJoystick = new Joystick(0);

  private static final String getShooterReady = "getShooterReady";
  private static final String Shoot = "Shoot";
  private static final String RevIntake = "Reverse Intake";
  private static final String Intake = "Intake";
  private static final String setModeSpeeker = "setModeSpeeker";
  private static final String setModeFeed = "setModeFeed";
  private static final String setModeAmp = "setModeAmp";
  private static final String setModeIntake = "setModeIntake";
  private static final String setModeDriveSlow = "setModeDriveSlow";
  private static final String setModeDriveNormal = "setModeDriveSlow";
  private static final String setModeDriveFast = "setModeDriveSlow";

  public IOSim() {
    SmartDashboard.putBoolean(getShooterReady, false);
    SmartDashboard.putBoolean(Shoot, false);
    SmartDashboard.putBoolean(RevIntake, false);
    SmartDashboard.putBoolean(Intake, false);
    SmartDashboard.putBoolean(setModeSpeeker, false);
    SmartDashboard.putBoolean(setModeFeed, false);
    SmartDashboard.putBoolean(getShooterReady, false);
    SmartDashboard.putBoolean(setModeAmp, false);
    SmartDashboard.putBoolean(setModeIntake, false);
    SmartDashboard.putBoolean(setModeDriveFast, false);
    SmartDashboard.putBoolean(setModeDriveNormal, false);
    SmartDashboard.putBoolean(setModeDriveSlow, false);
  }

  public DoubleSupplier getXPower() {
    return () -> simJoystick.getRawAxis(0);
  }

  public DoubleSupplier getYPower() {
    return () -> simJoystick.getRawAxis(1);
  }

  public DoubleSupplier getOmegaPower() {
    return () -> 0;
  }

  public Trigger getShooterReady() {
    return new Trigger(() -> SmartDashboard.getBoolean(getShooterReady, false));
  }
  ;

  public Trigger getShoot() {
    return new Trigger(() -> SmartDashboard.getBoolean(Shoot, false));
  }

  public Trigger ReverseIntake() {
    return new Trigger(() -> SmartDashboard.getBoolean(RevIntake, false));
  }

  public Trigger Intake() {
    return new Trigger(() -> SmartDashboard.getBoolean(Intake, false));
  }

  public Trigger setModeSpeeker() {
    return new Trigger(() -> SmartDashboard.getBoolean(setModeSpeeker, false));
  }

  public Trigger setModeAmp() {
    return new Trigger(() -> SmartDashboard.getBoolean(setModeAmp, false));
  }

  public Trigger setModeNote() {
    return new Trigger(() -> SmartDashboard.getBoolean(setModeIntake, false));
  }

  public Trigger setModeFeed() {
    return new Trigger(() -> SmartDashboard.getBoolean(setModeFeed, false));
  }

  public Trigger setAutoRotateOn() {
    return new Trigger(() -> SmartDashboard.getBoolean("NULL", false));
  }

  public Trigger setAutoRotateOff() {
    return new Trigger(() -> SmartDashboard.getBoolean("NULL", false));
  }

  public Trigger setPassiveSwitchOn() {
    return new Trigger(() -> SmartDashboard.getBoolean("NULL", false));
  }

  public Trigger setPassiveSwitchOff() {
    return new Trigger(() -> SmartDashboard.getBoolean("NULL", false));
  }

  public Trigger setDriveModeFast() {
    return new Trigger(() -> SmartDashboard.getBoolean(setModeDriveFast, false));
  }

  public Trigger setDriveModeNormal() {
    return new Trigger(() -> SmartDashboard.getBoolean(setModeDriveNormal, false));
  }

  public Trigger setDriveModeSlow() {
    return new Trigger(() -> SmartDashboard.getBoolean(setModeDriveSlow, false));
  }
}
