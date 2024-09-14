// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Passive.DriveCommands;
import frc.robot.commands.Passive.aimPassive;
import frc.robot.commands.Passive.shooterPassive;
import frc.robot.subsystems.IO.IOJoystick;
import frc.robot.subsystems.IO.IOMoudlue;
import frc.robot.subsystems.IO.IOSim;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.aim.Aim;
import frc.robot.subsystems.aim.aimMotorsIOKraken;
import frc.robot.subsystems.aim.aimMotorsIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeMotorsIOSim;
import frc.robot.subsystems.intake.IntakeMotorsIOVictor;
import frc.robot.subsystems.shooter.shooterSub.Shooter;
import frc.robot.subsystems.shooter.shooterSub.shooterMotorFalcon;
import frc.robot.subsystems.shooter.shooterSub.shooterRollerSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  SysIdRoutine.Direction bDirection;
  SysIdRoutine.Direction aDirection;

  public static Drive drive;
  public static Aim aim;
  public static Shooter shooter;
  public static Intake intake;
  public static final Vision vision = new Vision();

  private static IOMoudlue Contruller;

  //  private final OIbase OI = new OIGuitarAndJoystick(1,0);
  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case SIM:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        aim = new Aim(new aimMotorsIOSim());
        shooter = new Shooter(new shooterRollerSim());
        intake = new Intake(new IntakeMotorsIOSim());
        Contruller = new IOSim();

        break;

      default:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));

        aim = new Aim(new aimMotorsIOKraken());
        shooter = new Shooter(new shooterMotorFalcon());
        intake = new Intake(new IntakeMotorsIOVictor());
        Contruller = new IOJoystick();

        break;
    }

    Goals.setPositionSuppler(() -> drive.getPose());
    /*NamedCommands.registerCommand(
    "Run Flywheel",
    Commands.startEnd(
            () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
        .withTimeout(5.0));*/
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("autos", autoChooser);

    // Set up feedforward characterization
    // autoChooser.addOption(
    //   "Drive FF Characterization",
    //   new FeedForwardCharacterization(
    //      drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /*
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftX(),
            () -> -driverController.getLeftY(),
            () -> -driverController.getRightX()));*/

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, Contruller.getXPower(), Contruller.getYPower(), Contruller.getOmegaPower()));

    aim.setDefaultCommand(aimPassive.aimPassive(aim));
    shooter.setDefaultCommand(shooterPassive.shooterPassive(shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
