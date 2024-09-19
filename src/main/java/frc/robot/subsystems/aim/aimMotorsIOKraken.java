package frc.robot.subsystems.aim;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.MotorIDConst;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class aimMotorsIOKraken implements aimMotorsIO {

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

  public aimMotorsIOKraken() {
    leaderAimEncoder.setDistancePerRotation(1.0);
    followerAimEncoder.setDistancePerRotation(1.0);

    var aimingConfig = new TalonFXConfiguration();

    aimingConfig.Slot0.kP = ShooterConstants.AimKp;
    aimingConfig.Slot0.kD = ShooterConstants.AimKd;

    var aimMotionMagicConfigs = aimingConfig.MotionMagic;
    aimMotionMagicConfigs.MotionMagicCruiseVelocity = aimConstaints.MaxAccle;
    aimMotionMagicConfigs.MotionMagicAcceleration = aimConstaints.MaxVelocity;
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
  }

  public boolean setAngle(Rotation2d angle) {

    if (angle.getDegrees() > -65 && angle.getDegrees() < 65) {

      leaderAimingKraken.setControl(
          new MotionMagicVoltage(
              angle.getRotations() * ShooterConstants.kAimGearRatio,
              true,
              0.0,
              0,
              true,
              false,
              false));

      aimOutofLimit.set(false);
      return true;
    } else {

      aimOutofLimit.set(true);
      return false;
    }
  }
  ;

  public OutputAim getOutputs() {

    leaderAimPosition.refresh();
    leaderAimCurrent.refresh();
    leaderAimAppliedVolts.refresh();
    leaderAimVelocity.refresh();

    OutputAim outputAim = new OutputAim();

    outputAim.Current = leaderAimCurrent.getValueAsDouble();
    outputAim.Rotation =
        new Rotation2d(
            Rotations.of(leaderAimPosition.getValueAsDouble() / aimConstaints.kAimGearRatio));
    outputAim.Velocity = leaderAimVelocity.getValueAsDouble();
    outputAim.Volts = leaderAimAppliedVolts.getValueAsDouble();

    SmartDashboard.putNumber("LEncoder", leaderAimEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("FEncoder", followerAimEncoder.getAbsolutePosition());

    return outputAim;
  }
  ;

  public void setVolts(double Volts) {

    leaderAimingKraken.setVoltage(Volts);
    followerAimingKraken.setVoltage(Volts);
  }
  ;

  public void resetAimZeroPosition() {

    if (leaderAimEncoder.isConnected()) {
      var rotations =
          (leaderAimEncoder.getAbsolutePosition() - aimConstaints.kLeaderAimOffset)
              * aimConstaints.kAimGearRatio;
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

  public void stop() {
    leaderAimingKraken.stopMotor();
  }
}
