package frc.robot.subsystems.aim;

import edu.wpi.first.math.geometry.Rotation2d;

public class aimConstaints {

  public static final double AimKp = 0.6;
  public static final double AimKd = 0.0;

  public static final double MaxAccle = 150;
  public static final double MaxVelocity = 300;

  public static final double kAimGearRatio = 240.428571429;
  public static final double kLeaderAimOffset = 0.503;
  public static final double kFollowerAimOffset = 0.512;

  public static final Rotation2d PassiveAmpAngle = new Rotation2d(0);
  public static final Rotation2d FeedAngle = new Rotation2d(Math.PI / 4);
}
