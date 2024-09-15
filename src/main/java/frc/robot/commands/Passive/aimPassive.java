package frc.robot.commands.Passive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Goals;
import frc.robot.subsystems.aim.Aim;
import frc.robot.subsystems.aim.aimConstaints;

public class aimPassive {

  // The avarage shooter angle when note shoot at Speeker
  private static Rotation2d ShooterAvg = new Rotation2d(Degree.of(35));

  // 45 deg
  private static final Rotation2d shooterFeedAngle = aimConstaints.FeedAngle;

  // The avarage shooter angle when note shoot
  private static Rotation2d ShootAvgAngle = new Rotation2d(Degree.of((35 + 45) / 2));

  public static Command aimPassive(Aim aim) {
    return Commands.run(
        () -> {
          SmartDashboard.putNumber("Shooter Avg", ShooterAvg.getDegrees());
          SmartDashboard.putNumber("Shooter avg angle", ShootAvgAngle.getDegrees());

          switch (Goals.getGoalInfo().goal) {
            case FEED:
              aim.aimShooter(shooterFeedAngle);
              break;

            case SPEEKER:
              aim.aimShooter(ShooterAvg);
              break;

            default:
              aim.aimShooter(ShootAvgAngle);
              break;
          }
        },
        aim);
  }

  public static void addShootSpeekerAngle(Rotation2d angle) {

    ShooterAvg.plus(angle).div(2);
    addShootAvgAngle(angle);
  }

  public static void addShootAvgAngle(Rotation2d angle) {

    ShootAvgAngle.plus(angle).div(2);
  }
}
