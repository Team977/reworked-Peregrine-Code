// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.MotorIDConst;

public class MagRoller extends SubsystemBase {

  private final TalonFX leaderMagFalcon =
      new TalonFX(MotorIDConst.ShooterleaderMagFalconID, Constants.CANivore);
  private final TalonFX followerMagFalcon =
      new TalonFX(MotorIDConst.ShooterfollowerMagFalconID, Constants.CANivore);

  private final StatusSignal<Double> leaderMagPosition = leaderMagFalcon.getPosition();
  private final StatusSignal<Double> leaderMagVelocity = leaderMagFalcon.getVelocity();
  private final StatusSignal<Double> leaderMagAppliedVolts = leaderMagFalcon.getMotorVoltage();
  private final StatusSignal<Double> leaderMagCurrent = leaderMagFalcon.getStatorCurrent();
  private final StatusSignal<Double> followerMagCurrent = followerMagFalcon.getStatorCurrent();

  /** Creates a new Shooter. */
  public MagRoller() {

    var magConfig = new TalonFXConfiguration();
    magConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    magConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    magConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    magConfig.Slot0.kP = .5;
    leaderMagFalcon.getConfigurator().apply(magConfig);
    followerMagFalcon.getConfigurator().apply(magConfig);
    followerMagFalcon.setControl(new Follower(leaderMagFalcon.getDeviceID(), true));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderMagPosition,
        leaderMagVelocity,
        leaderMagAppliedVolts,
        leaderMagCurrent,
        followerMagCurrent);
  }

  @Override
  public void periodic() {}

  /** Set the speed of the mag roller -1.0 to 1.0 */
  public void setMagSpeed(double speed) {
    leaderMagFalcon.setControl(new DutyCycleOut(speed, true, false, false, false));
  }

  public void resetMagPosition() {
    leaderMagFalcon.setPosition(0.0);
  }

  /** Set the position of the mag roller */
  public void setMagPosition(double position) {
    leaderMagFalcon.setControl(
        new PositionVoltage(position, 0.0, true, 0.0, 0, false, false, false));
  }

  public void stopMag() {
    leaderMagFalcon.stopMotor();
  }
}
