// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Goals;
import frc.robot.Math977;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {

          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          linearMagnitude = addTranslationMod(linearMagnitude);
          omega = addRotationMod(omega);

          /*if (Goals.getGoalInfo().AutoRotate && omega <= 0.1) {
            omega = getAutoTurnPower(drive);
          }*/

          // Calculate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped = Math977.isRed();
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  private static final ProfiledPIDController PID =
      new ProfiledPIDController(1, 0, 0, new Constraints(.75, .2), 0.02);

  private static double getAutoTurnPower(Drive drive) {

    // get Rotations
    Rotation2d DesiredRotation = getDiseredAutoRotationOffset(drive.getPose());

    return PID.calculate(DesiredRotation.getRotations(), 0);
  }

  private static Rotation2d getDiseredAutoRotationOffset(Pose2d Robot) {
    switch (Goals.getGoalInfo().goal) {
      case SPEEKER:
        return getAngleBetweenRobotAndSpeeker(Robot.getTranslation());

      case INTAKE:
        return getAngleBetweenRobotAndNote(Robot);

      case FEED:
        return getAngleOffsetToFeedRotation(Robot);

      case AMP:
        return getAngleOffsetToAmp(Robot);

      default:
        return new Rotation2d(0);
    }
  }

  private static Rotation2d getAngleBetweenRobotAndSpeeker(Translation2d Robot) {
    return Math977.getRotationBetweanSpeekerAndRobotYaw(new Translation3d(Robot.getX(), Robot.getY(), 0));
  }

  private static Rotation2d getAngleBetweenRobotAndNote(Pose2d Robot) {
    return new Rotation2d();
  }

  private static Rotation2d getAngleOffsetToAmp(Pose2d Robot) {
    return new Rotation2d();
  }

  private static Rotation2d getAngleOffsetToFeedRotation(Pose2d Robot) {
    return new Rotation2d();
  }

  private static double addTranslationMod(double input) {
    return input * Goals.getTranslationMod();
  }

  private static double addRotationMod(double input) {
    return input * Goals.getRotationnMod();
  }
}
