// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.MotorIDConst;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Aim extends SubsystemBase {

  private final DutyCycleEncoder leaderAimEncoder = new DutyCycleEncoder(0);
  private final DutyCycleEncoder followerAimEncoder = new DutyCycleEncoder(1);

  private final TalonFX leaderAimingKraken =
      new TalonFX(MotorIDConst.ShooterleaderAimingKrakenID, Constants.CANivore);
  private final TalonFX followerAimingKraken =
      new TalonFX(MotorIDConst.ShooterfollowerAimingKrakenID, Constants.CANivore);

  private final StatusSignal<Double> leaderAimPosition = leaderAimingKraken.getPosition();
  private final StatusSignal<Double> leaderAimVelocity = leaderAimingKraken.getVelocity();
  private final StatusSignal<Double> leaderAimAppliedVolts = leaderAimingKraken.getMotorVoltage();
  private final StatusSignal<Double> leaderAimCurrent = leaderAimingKraken.getStatorCurrent();
  private final StatusSignal<Double> followerAimCurrent = followerAimingKraken.getStatorCurrent();

  private Alert noLeadEncoder =
      new Alert("Lead Aim Encoder not found, using follower encoder", AlertType.WARNING);
  private Alert noFollowerEncoder =
      new Alert("Both encoders not found, Aim motors not reset", AlertType.WARNING);

  private Alert aimOutofLimit = new Alert("Attempted to aim outside of limits", AlertType.ERROR);

  private boolean ShooterHasNote = false;

  /** Creates a new Shooter. */
  public Aim() {

    leaderAimEncoder.setDistancePerRotation(1.0);
    followerAimEncoder.setDistancePerRotation(1.0);

    var aimingConfig = new TalonFXConfiguration();

    aimingConfig.Slot0.kP = ShooterConstants.AimKp;
    aimingConfig.Slot0.kD = ShooterConstants.AimKd;

    var aimMotionMagicConfigs = aimingConfig.MotionMagic;
    aimMotionMagicConfigs.MotionMagicCruiseVelocity = 150;
    aimMotionMagicConfigs.MotionMagicAcceleration = 300;
    aimMotionMagicConfigs.MotionMagicJerk = 2000;

    aimingConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    aimingConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    aimingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leaderAimingKraken.getConfigurator().apply(aimingConfig);
    followerAimingKraken.getConfigurator().apply(aimingConfig);
    followerAimingKraken.setControl(new Follower(leaderAimingKraken.getDeviceID(), true));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderAimPosition,
        leaderAimVelocity,
        leaderAimAppliedVolts,
        leaderAimCurrent,
        followerAimCurrent);
    leaderAimingKraken.optimizeBusUtilization();
    followerAimingKraken.optimizeBusUtilization();

    resetAimZeroPosition();

    SmartDashboard.putNumber("aim angle", -35.0);
    SmartDashboard.putNumber("april tag offset", 20.5);
    SmartDashboard.putNumber("Shoot speed", 4);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Angle", getShooterAngle());

    SmartDashboard.putNumber("Aim L Encoder", leaderAimEncoder.getAbsolutePosition()); // 1-this
    SmartDashboard.putNumber("Aim F Encoder", followerAimEncoder.getAbsolutePosition());

    // SmartDashboard.putNumber("Aim Rotations",
    // leaderAimingKraken.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
  }

  /** Sends a control request to move the shooter to the given angle */
  public void aimShooter(double degrees) {

    Logger.recordOutput("Shooter setPoint", degrees);
    Logger.recordOutput("Shooter Angle", getShooterAngle());
    Logger.recordOutput("Shooter Vell", leaderAimVelocity.getValueAsDouble());

    if (degrees > -70 && degrees < 45) {
      setAimPosition(degrees / 360 * ShooterConstants.kAimGearRatio);

      aimOutofLimit.set(false);
    } else {
      SmartDashboard.putNumber("Aim Outside Limits Last Value", degrees);
      aimOutofLimit.set(true);
    }
  }

  /** Sends a control request to move the shooter to the given angle */
  public double getAimShooterValue(double degrees) {
    return degrees / 360 * ShooterConstants.kAimGearRatio;
  }

  /**
   * Reads the Duty Cycle Encoder, converts that to rotations and resets the motor position to match
   */
  public void resetAimZeroPosition() {

    if (leaderAimEncoder.isConnected()) {
      var rotations =
          (leaderAimEncoder.getAbsolutePosition() - ShooterConstants.kLeaderAimOffset)
              * ShooterConstants.kAimGearRatio;
      leaderAimingKraken.setPosition(-rotations);
      followerAimingKraken.setPosition(-rotations);
      noLeadEncoder.set(false);
      noFollowerEncoder.set(false);
      SmartDashboard.putNumber("Reset Rotation", -rotations);
    } else if (followerAimEncoder.isConnected()) {

      noLeadEncoder.set(true);
      var rotations =
          (followerAimEncoder.getAbsolutePosition() - ShooterConstants.kFollowerAimOffset)
              * ShooterConstants.kAimGearRatio;
      leaderAimingKraken.setPosition(-rotations);
      followerAimingKraken.setPosition(-rotations);
    } else {

      noFollowerEncoder.set(true);
    }
  }

  /** Sets the aim position using motion magic voltage control */
  public void setAimPosition(double positionAim) {
    leaderAimingKraken.setControl(
        new MotionMagicVoltage(positionAim, true, 0.0, 0, true, false, false));
  }

  /** Stops the aiming motors */
  public void stopAim() {
    leaderAimingKraken.stopMotor();
  }

  public double getShooterAngle() {
    return leaderAimEncoder.getAbsolutePosition() / ShooterConstants.GearRatio;
  }

  public boolean CheckIfShooterHasNote() {
    return ShooterHasNote;
  }

  public boolean angleAtTarget(double target) {

    double temp =
        Math.abs(getAimShooterValue(target) - leaderAimingKraken.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("Aim Error", temp);

    return temp < 0.05;
  }

  public void HasNote() {
    ShooterHasNote = true;
  }

  public boolean PassedAngle(double Angle) {
    return Math.abs(getShooterAngle()) > Angle;
  }
}
