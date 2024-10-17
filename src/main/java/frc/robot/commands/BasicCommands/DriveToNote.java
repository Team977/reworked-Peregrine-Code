// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Passive.DriveCommands;
import frc.robot.subsystems.drive.Drive;

public class DriveToNote extends Command {

  Drive drive;
  double MaxDistance;
  Translation2d StartPose;
  double Speed;
  /** Creates a new DriveToRotation. */
  public DriveToNote(Drive drive, double MaxDistance, double Speed) {
    this.drive = drive;
    this.MaxDistance = MaxDistance;
    this.Speed = Speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    StartPose = drive.getPose().getTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double omega = DriveCommands.getPowerOfRotationToNote(drive.getPose());

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            Speed * drive.getMaxLinearSpeedMetersPerSec(),
            0,
            omega * drive.getMaxAngularSpeedRadPerSec(),
            new Rotation2d(0)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (StartPose.getDistance(drive.getPose().getTranslation()) >= MaxDistance) {
      return true;
    }
    return false;
  }
}
