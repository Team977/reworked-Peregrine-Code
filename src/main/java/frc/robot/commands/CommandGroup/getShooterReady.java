// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroup;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Goals;
import frc.robot.Goals.Goal;
import frc.robot.Math977;
import frc.robot.RobotContainer;
import frc.robot.commands.BasicCommands.AngleShooter;
import frc.robot.commands.BasicCommands.RunIntake;
import frc.robot.commands.BasicCommands.RunShooter;
import frc.robot.subsystems.aim.Aim;
import frc.robot.subsystems.aim.aimConstaints;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeSub.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class getShooterReady extends ParallelCommandGroup {
  /** Creates a new getShooterReady. */
  public getShooterReady(Drive drive, Aim aim, Shooter shooter, Intake intake) {

    Translation3d Speeker =
        Math977.isRed() ? Constants.Vision.SpeekerBlue : Constants.Vision.SpeekerRed;

    /*
     *
     *
     * get angle
     *
     * if Speeker:
     *    angle at speeker
     *
     * if Feed:
     *    angle at 45
     *
     */

    Supplier<Rotation2d> shooterAngle =
        () ->
            Goals.getGoalInfo().goal == Goal.SPEEKER
                ?
                // new Rotation2d(Units.Degrees.of(-35))
                // Speeker
                new Rotation2d(
                        Math.atan2(
                            Speeker.getZ(),
                            RobotContainer.drive
                                .getPose()
                                .getTranslation()
                                .getDistance(new Translation2d(Speeker.getX(), Speeker.getY()))))
                    .minus(new Rotation2d(Math.PI / 2))
                // new Rotation2d(Units.Degrees.of(-35))
                :
                // FEED, 45
                aimConstaints.FeedAngle;

    addCommands(

        // aim shooter
        new AngleShooter(aim, shooterAngle),

        // run shooter back so note note in shooter motor
        new RunNoteBack(shooter, intake)
            .andThen(

                // spin up shooter
                new RunShooter(
                    shooter,
                    Goals.getGoalInfo().goal == Goal.SPEEKER
                        ? ShooterConstants.SpeekerShooterSpeed
                        : ShooterConstants.FeedShooterSpeed)));
  }
}
