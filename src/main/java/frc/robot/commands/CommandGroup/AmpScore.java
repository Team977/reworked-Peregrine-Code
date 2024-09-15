// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroup;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.BasicCommands.AngleShooter;
import frc.robot.commands.BasicCommands.RunIntake;
import frc.robot.commands.BasicCommands.RunShooter;
import frc.robot.subsystems.aim.Aim;
import frc.robot.subsystems.aim.aimConstaints;
import frc.robot.subsystems.intake.IntakeSub.Intake;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScore extends ParallelCommandGroup {
  /** Creates a new AmpScore. */
  public AmpScore(Aim aim, Shooter shooter, Intake intake) {

    Rotation2d shootRotation = new Rotation2d(0);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.deadline(
            Commands.waitUntil(() -> aim.atPosition(shootRotation, new Rotation2d(Math.PI / 15)))
                .andThen(new RunShooter(shooter, 4).alongWith(new RunIntake(intake, .5)))
                .withTimeout(1),
            new AngleShooter(aim, () -> aimConstaints.PassiveAmpAngle.times(-1))));
  }
}
