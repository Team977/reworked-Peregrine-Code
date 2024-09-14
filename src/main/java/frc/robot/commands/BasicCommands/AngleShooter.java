// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.aim.Aim;
import java.util.function.Supplier;

public class AngleShooter extends Command {

  private final Aim aim;

  private final Supplier<Rotation2d> angle;

  private final boolean Stop;

  private final Rotation2d Treashold;

  /** Creates a new AngleShooter. */
  public AngleShooter(Aim aim, Supplier<Rotation2d> DesiredAngle) {

    this.aim = aim;
    this.angle = DesiredAngle;
    this.Stop = false;

    Treashold = new Rotation2d(0);

    addRequirements(this.aim);
  }

  /** Creates a new AngleShooter. */
  public AngleShooter(Aim aim, Supplier<Rotation2d> DesiredAngle, Rotation2d Threashold) {

    this.aim = aim;
    this.angle = DesiredAngle;
    this.Stop = true;

    this.Treashold = Threashold;

    addRequirements(this.aim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    aim.aimShooter(angle.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    aim.STOP();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!Stop) {
      return false;
    }
    return aim.atPosition(angle.get(), Treashold);
  }
}
