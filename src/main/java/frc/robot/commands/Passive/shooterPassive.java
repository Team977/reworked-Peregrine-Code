package frc.robot.commands.Passive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Goals;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class shooterPassive {

  private static final double SpeedMod = 0.1;

  public static Command shooterPassive(Shooter shooter) {
    return Commands.run(
        () -> {
          switch (Goals.getGoalInfo().goal) {
            case SPEEKER:
              shooter.setVelocity(ShooterConstants.SpeekerShooterSpeed * SpeedMod);
              break;

            case FEED:
              shooter.setVelocity(ShooterConstants.FeedShooterSpeed * SpeedMod);
              break;

            default:
              shooter.setVelocity(0);
              break;
          }
        },
        shooter);
  }
}
