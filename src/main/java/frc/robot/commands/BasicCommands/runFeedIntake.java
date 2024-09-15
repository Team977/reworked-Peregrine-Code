// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.feedIntake.FeedIntake;

public class runFeedIntake extends Command {

  private final FeedIntake feedIntake;
  private final double speed;

  /** Creates a new runFeedIntake. */
  public runFeedIntake(FeedIntake feedIntake, double speed) {
    this.feedIntake = feedIntake;
    this.speed = speed;

    addRequirements(feedIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feedIntake.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feedIntake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}