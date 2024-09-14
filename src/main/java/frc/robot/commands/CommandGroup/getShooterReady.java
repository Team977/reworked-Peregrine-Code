// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroup;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.BasicCommands.AngleShooter;
import frc.robot.commands.BasicCommands.RunShooter;
import frc.robot.subsystems.aim.Aim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.shooterSub.Shooter;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class getShooterReady extends ParallelCommandGroup {
  /** Creates a new getShooterReady. */
  public getShooterReady(Drive drive, Aim aim, Shooter shooter) {

    Supplier<Rotation2d> shooterAngle =
        () ->
            new Rotation2d(
                    drive
                        .getPose()
                        .getTranslation()
                        .getDistance(Constants.Vision.SpeekerBlue.toTranslation2d()),
                    Constants.Vision.SpeekerBlue.getZ())
                .minus(new Rotation2d(Math.PI / 2));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AngleShooter(aim, shooterAngle), new RunShooter(shooter, 18));
  }
}
