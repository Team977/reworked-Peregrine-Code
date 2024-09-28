// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.aim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Math977;
import frc.robot.RobotContainer;
import frc.robot.subsystems.aim.aimMotorsIO.OutputAim;

public class Aim extends SubsystemBase {

  private final aimMotorsIO AimMotorsIO;
  private final SysIdRoutine sysId;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm;

  Rotation2d DesiredAngle = new Rotation2d(0);

  /** Creates a new Aim. */
  public Aim(aimMotorsIO AimMotorsIO) {

    this.AimMotorsIO = AimMotorsIO;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  AimMotorsIO.setVolts(voltage.in(Volts));
                },
                log -> {
                  OutputAim logout = AimMotorsIO.getOutputs();

                  log.motor("FL")
                      .angularPosition(Rotations.of(logout.Rotation.getRotations()))
                      .angularVelocity(RotationsPerSecond.of(logout.Velocity))
                      .current(Amps.of(logout.Current))
                      .voltage(Volts.of(logout.Volts));
                  // .linearPosition(logout.Position)
                  // .linearVelocity(logout.Velocity);
                },
                this));

    m_arm =
        m_armPivot.append(
            new MechanismLigament2d(
                "Arm",
                30,
                AimMotorsIO.getOutputs().Rotation.getDegrees() + 90,
                6,
                new Color8Bit(Color.kYellow)));

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));
  }

  public void aimShooter(Rotation2d rotation) {

    DesiredAngle = rotation;

    if (rotation.getDegrees() > 50 || rotation.getDegrees() < -50) {
      return;
    }

    AimMotorsIO.setAngle(rotation);
  }

  public Rotation2d getAngle() {
    return AimMotorsIO.getOutputs().Rotation;
  }

  public boolean atPosition(Rotation2d angle, Rotation2d treashould) {

    return Math.abs(angle.getDegrees() - getAngle().getDegrees()) < treashould.getDegrees();
  }

  public boolean atPosition(Rotation2d treashould) {

    double ShooterAngle = DesiredAngle.getDegrees();

    return getAngle().getDegrees() > ShooterAngle - treashould.getDegrees()
        && getAngle().getDegrees() < ShooterAngle + treashould.getDegrees();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public void STOP() {
    AimMotorsIO.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    OutputAim outputAim = AimMotorsIO.getOutputs();

    m_arm.setAngle(new Rotation2d(outputAim.Rotation.getRadians() + Math.PI / 2));

    SmartDashboard.putNumber("aim angle deg", outputAim.Rotation.getDegrees());
    SmartDashboard.putNumber("aim current", outputAim.Current);
    SmartDashboard.putNumber("aim Vel deg", outputAim.Velocity * 360);
    SmartDashboard.putNumber("aim Volts", outputAim.Volts);
    SmartDashboard.putNumber("Desired angle Deg", DesiredAngle.getDegrees());

    Translation3d Speeker =
        Math977.isRed() ? Constants.Vision.SpeekerRed : Constants.Vision.SpeekerBlue;

    SmartDashboard.putNumber(
        "Auto Aim",
        new Rotation2d(
                Math.atan2(
                    Speeker.getZ(),
                    RobotContainer.drive
                        .getPose()
                        .getTranslation()
                        .getDistance(new Translation2d(Speeker.getX(), Speeker.getY()))))
            .minus(new Rotation2d(Math.PI / 2))
            .getDegrees());
  }
}
