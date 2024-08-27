// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.mag;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.mag.magMotorIO.OutputMag;

public class MagRoller extends SubsystemBase {

  private final magMotorIO MagMotorIO;

  private double desiredSpeed;

  /** Creates a new Shooter. */
  public MagRoller(magMotorIO i_magMotorIO) {

    MagMotorIO = i_magMotorIO;
  }

  @Override
  public void periodic() {

    OutputMag outputMag = MagMotorIO.getOutput();

    SmartDashboard.putNumber("mag Velocity", outputMag.speed);
    SmartDashboard.putNumber("mag Volts", outputMag.volts);
    SmartDashboard.putNumber("mag Amps", outputMag.amps);
    SmartDashboard.putNumber("mag Desired Speed", desiredSpeed);
  }

  /** Set the speed of the mag roller -1.0 to 1.0 */
  public void setMagSpeed(double speed) {

    desiredSpeed = speed * 6380;
    MagMotorIO.setSpeed(speed);
  }

  public double getSpeed() {
    return MagMotorIO.getOutput().speed;
  }
}
