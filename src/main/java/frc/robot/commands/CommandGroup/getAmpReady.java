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
public class getAmpReady extends ParallelCommandGroup {
  /** Creates a new getAmpReady. */
  public getAmpReady(Intake intake, Shooter shooter, Aim aim) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        //aim shooter
        new AngleShooter(aim, () -> aimConstaints.PassiveAmpAngle, new Rotation2d(Math.PI / 30)),

        // along with

        // once the note is not seen stop
        Commands.deadline(

            //run Intake and Shooter UNTIL note is not seen
            Commands.waitUntil(() -> !intake.isNotePresent()),
                new RunIntake(intake, .2).repeatedly(),
                new RunShooter(shooter, 1)
          )

        // once note is not seen run shooter for a secound
        .andThen(new RunShooter(shooter, 1).withTimeout(.2)));
  }
}
