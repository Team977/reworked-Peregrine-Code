package frc.robot.subsystems.aim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

public class aimConstaints {

  public static final double AimKp = 0.6;
  public static final double AimKd = 0.0;

  public static final double MaxAccle = 150;
  public static final double MaxVelocity = 300;

  public static final double kAimGearRatio = 240.428571429;
  public static final double kFollowerAimOffset = 0.851; // 0.841;
  public static final double kLeaderAimOffset = .505; // 0.684;

  public static final Rotation2d PassiveAmpAngle = new Rotation2d(Units.Degrees.of(-25)); // 30][\]
  public static final Rotation2d FeedAngle = new Rotation2d(Units.Degrees.of(-35));
}
