// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.aim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.aim.aimMotorsIO.OutputAim;

public class Aim extends SubsystemBase {

  private final aimMotorsIO AimMotorsIO;
  private final SysIdRoutine sysId;

  Rotation2d DesiredAngle = new Rotation2d(0);

  /** Creates a new Aim. */
  public Aim(aimMotorsIO AimMotorsIO) {

    this.AimMotorsIO = AimMotorsIO;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  AimMotorsIO.setVolts(voltage.in(Volts));
                },
                log -> {
                  OutputAim logout = AimMotorsIO.getOutputs();

                  log.motor("FL")
                      .angularPosition(Rotations.of(logout.Rotation.getRotations()))
                      .angularVelocity(RotationsPerSecond.of(logout.Velocity))
                      .current(Amps.of(logout.Current))
                      .voltage(Volts.of(logout.Volts));
                  // .linearPosition(logout.Position)
                  // .linearVelocity(logout.Velocity);
                },
                this));
  }

  public void aimShooter(Rotation2d rotation) {
    AimMotorsIO.setAngle(rotation);

    DesiredAngle = rotation;
  }

  public Rotation2d getAngle() {
    return AimMotorsIO.getOutputs().Rotation;
  }

  public boolean atPosition(Rotation2d angle, Rotation2d treashould) {

    double ShooterAngle = angle

    return getAngle().getDegrees() > ShooterAngle - treashould.getDegrees()
        && getAngle().getDegrees() < ShooterAngle + treashould.getDegrees();
  }

  public boolean atPosition(Rotation2d treashould) {

    double ShooterAngle = DesiredAngle.getDegrees();

    return getAngle().getDegrees() > ShooterAngle - treashould.getDegrees()
        && getAngle().getDegrees() < ShooterAngle + treashould.getDegrees();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    OutputAim outputAim = AimMotorsIO.getOutputs();

    SmartDashboard.putNumber("aim angle deg", outputAim.Rotation.getDegrees());
    SmartDashboard.putNumber("aim current", outputAim.Current);
    SmartDashboard.putNumber("aim Vel deg", outputAim.Velocity * 360);
    SmartDashboard.putNumber("aim Volts", outputAim.Volts);
    SmartDashboard.putNumber("Desired angle Deg", DesiredAngle.getDegrees());
  }
}
