// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroup;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Goals;
import frc.robot.Goals.Goal;
import frc.robot.RobotContainer;
import frc.robot.commands.BasicCommands.RunIntake;
import frc.robot.commands.BasicCommands.RunShooter;
import frc.robot.commands.BasicCommands.runFeedIntake;
import frc.robot.subsystems.intake.IntakeSub.Intake;
import frc.robot.subsystems.intake.feedIntake.FeedIntake;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSequence extends ParallelCommandGroup {
  /** Creates a new IntakeSequence. */
  public IntakeSequence(FeedIntake feedIntake, Intake intake, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        // once run intake finshes go forward
        new RunIntake(intake, .65)
            .stop()
            .deadlineWith(new runFeedIntake(feedIntake, -.75))

            // pull note into shooter slowly
            .andThen(
                new RunIntake(intake, .2)
                    .repeatedly()
                    .alongWith(
                        new RunShooter(shooter, () -> -1),
                        Commands.runOnce(() -> RobotContainer.CANdle.pickedUpNote()),
                        Commands.runOnce(() -> Goals.ChangeGoal(Goal.SPEEKER)))
                    .withTimeout(0.2)
                    .andThen(new RunShooter(shooter, () -> 0))));
  }
}
