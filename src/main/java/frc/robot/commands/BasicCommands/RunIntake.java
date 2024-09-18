// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSub.Intake;

public class RunIntake extends Command {

  private final Intake intake;
  private final double speed;

  private boolean stop = false;

  /** Creates a new RunIntake. */
  public RunIntake(Intake intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intake = intake;
    this.speed = speed;

    addRequirements(intake);
  }

  public RunIntake stop() {
    stop = true;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeNote(speed * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intake.intakeNote(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.isNotePresent() && stop;
  }
}
