package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConstants {

  // rps
  public static final double SpeekerShooterSpeed = 4;
  // rps
  public static final double FeedShooterSpeed = 4;

  public static final Rotation2d PassiveAmpAngle = new Rotation2d(0);

  public static final double ShooterMagRollerSpeed = -1.0;
  public static final double IntakeMagRollerSpeed = -0.5;

  public static final double GearRatio = 240.428571429;

  public static final double AimKp = 0.6;
  public static final double AimKd = 0.0;

  public static final double ShooterKp = 0.08;
  public static final double ShooterKd = 0.00;

  public static final double kAimGearRatio = 240.428571429;
  public static final double kLeaderAimOffset = 0.503;
  public static final double kFollowerAimOffset = 0.512;
}
