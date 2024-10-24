// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.ModuleIO.LogOut;
import frc.robot.subsystems.shooter.shooterMotorIO.OutputShooter;

public class Shooter extends SubsystemBase {

  private final shooterMotorIO ShooterMotorIO;
  private double desiredSpeed;

  private SysIdRoutine sysIdRoutine;
  /** Creates a new Shooter. */
  public Shooter(shooterMotorIO ShooterMotorIO) {

    this.ShooterMotorIO = ShooterMotorIO;

    sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    ShooterMotorIO.setVolts(voltage.in(Units.Volts));
                },
                log -> {
                  OutputShooter logout = ShooterMotorIO.getOutput();

                  log.motor("Leed Motor")
                      .angularVelocity(Units.RotationsPerSecond.of(logout.speed))
                      .angularPosition(Units.Rotations.of(logout.position))
                      .current(Units.Amps.of(logout.amps))
                      .voltage(Units.Volts.of(logout.volts));
                  // .linearPosition(logout.Position)
                  // .linearVelocity(logout.Velocity);
                },
                this));
  }

  @Override
  public void periodic() {
    OutputShooter outputShooter = ShooterMotorIO.getOutput();

    SmartDashboard.putNumber("shooter Velocity", outputShooter.speed);
    SmartDashboard.putNumber("shooter Volts", outputShooter.volts);
    SmartDashboard.putNumber("shooter Amps", outputShooter.amps);
    SmartDashboard.putNumber("shooter Desired Speed", desiredSpeed);
  }

  public void setVelocity(double speed) {
    desiredSpeed = speed;
    ShooterMotorIO.setVelocity(speed, speed);
  }

  public double getVelocity() {
    return ShooterMotorIO.getOutput().speed;
  }

  public Command RunSysIDTest(boolean quasistatic){
    double time = 15;

    if (quasistatic) {
    
      sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(time)
    .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(time));
    
    }

    return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(time)
    .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(time));

  }
}
