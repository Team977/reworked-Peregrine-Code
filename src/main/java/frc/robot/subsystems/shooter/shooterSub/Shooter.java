// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.shooterSub;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooterSub.shooterMotorIO.OutputShooter;

public class Shooter extends SubsystemBase {

  private final shooterMotorIO ShooterMotorIO;
  private double desiredSpeed;
  /** Creates a new Shooter. */
  public Shooter(shooterMotorIO ShooterMotorIO) {

    this.ShooterMotorIO = ShooterMotorIO;
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
}
