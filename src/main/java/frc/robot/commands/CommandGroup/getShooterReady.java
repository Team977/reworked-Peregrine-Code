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
import frc.robot.commands.BasicCommands.AngleShooter;
import frc.robot.commands.BasicCommands.RunShooter;
import frc.robot.subsystems.aim.Aim;
import frc.robot.subsystems.aim.aimConstaints;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.shooterSub.Shooter;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class getShooterReady extends ParallelCommandGroup {
  /** Creates a new getShooterReady. */
  public getShooterReady(Drive drive, Aim aim, Shooter shooter) {

    Translation3d Speeker =
        Math977.isRed() ? Constants.Vision.SpeekerRed : Constants.Vision.SpeekerBlue;

    Supplier<Rotation2d> shooterAngle =
        () ->
            Goals.getGoalInfo().goal == Goal.SPEEKER
                ?
                // Speeker
                new Rotation2d(
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(new Translation2d(Speeker.getX(), Speeker.getY())),
                        Speeker.getZ())
                    .minus(new Rotation2d(Math.PI / 2))
                    .times(-1)
                :
                // FEED, 45
                aimConstaints.FeedAngle;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AngleShooter(aim, shooterAngle),
        new RunShooter(
            shooter,
            Goals.getGoalInfo().goal == Goal.SPEEKER
                ? ShooterConstants.SpeekerShooterSpeed
                : ShooterConstants.FeedShooterSpeed));
  }
}
