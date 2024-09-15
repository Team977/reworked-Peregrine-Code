// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.IntakeSub.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends Command {

  private final Shooter shooter;
  private final Intake intake;

  private final double shooterSpeed;
  private final double intakeSpeed;

  /** Creates a new Shoot. */
  public Shoot(
      Shooter shooter, Intake intake, double shooterSpeed, double intakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooter = shooter;
    this.intake = intake;
    this.intakeSpeed = intakeSpeed;
    this.shooterSpeed = shooterSpeed;

    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intake.intakeNote(intakeSpeed);
    shooter.setVelocity(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
