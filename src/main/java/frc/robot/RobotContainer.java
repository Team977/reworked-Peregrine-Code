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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Goals.DriveMode;
import frc.robot.Goals.Goal;
import frc.robot.commands.BasicCommands.RunShooter;
import frc.robot.commands.CommandGroup.IntakeSequence;
import frc.robot.commands.CommandGroup.RunNoteBack;
import frc.robot.commands.CommandGroup.Shoot;
import frc.robot.commands.CommandGroup.getShooterReady;
import frc.robot.commands.Passive.aimPassive;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.IO.IOMoudlue;
import frc.robot.subsystems.IO.IOXboxCon;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.aim.Aim;
import frc.robot.subsystems.aim.aimConstaints;
import frc.robot.subsystems.aim.aimMotorsIOKraken;
import frc.robot.subsystems.aim.aimMotorsIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.intake.IntakeSub.Intake;
import frc.robot.subsystems.intake.IntakeSub.IntakeMotorsIOSim;
import frc.robot.subsystems.intake.IntakeSub.IntakeMotorsIOVictor;
import frc.robot.subsystems.intake.feedIntake.FeedIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.shooterMotorFalcon;
import frc.robot.subsystems.shooter.shooterRollerSim;

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
  public static FeedIntake feedIntake;
  private static Candle CANdle = new Candle();

  private static final CommandXboxController test = new CommandXboxController(0);
  private static final CommandXboxController OpTest = new CommandXboxController(1);

  private static IOMoudlue Contruller = new IOXboxCon();

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
        // Contruller = new IOSim();
        feedIntake = new FeedIntake();

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
        // Contruller = new IOJoystick();
        feedIntake = new FeedIntake();

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

    // drive.setDefaultCommand(
    //    DriveCommands.joystickDrive(
    //        drive, Contruller.getXPower(), Contruller.getYPower(), Contruller.getOmegaPower()));

    aim.setDefaultCommand(aimPassive.aimPassive(aim));
    // shooter.setDefaultCommand(shooterPassive.shooterPassive(shooter));

    Command intakeSequence = new IntakeSequence(feedIntake, intake, shooter);
    Command getShooterReady = new getShooterReady(drive, aim, shooter, intake);
    Command getAmpReady = new RunNoteBack(shooter, intake);
    /*new AngleShooter(
        aim, () -> aimConstaints.PassiveAmpAngle, new Rotation2d(Units.Degrees.of(1)))
    .andThen(new RunShooter(shooter, 1.5).alongWith(new RunIntake(intake, 1)));*/
    Command setPoseAtSpeeker = Commands.runOnce(() -> drive.StepPoseAtSpeeker(1.5));
    Command ShootAmp = new Shoot(shooter, intake, 1.5, 1);
    Command Shoot = new Shoot(shooter, intake, ShooterConstants.SpeekerShooterSpeed, 1);
    Command MannuleGetShooterReady =
        new getShooterReady(drive, aim, shooter, intake, new Rotation2d(Units.Degrees.of(-25)));
    Command FeedGetShooterReady =
        new getShooterReady(drive, aim, shooter, intake, aimConstaints.FeedAngle);
    Command stopShooter = new RunShooter(shooter, 0);
    Command RevIntake = new Shoot(shooter, intake, -1, -.1);

    Contruller.Intake().whileTrue(stopShooter);
    Contruller.getShooterReady()
        .whileTrue(
            new ConditionalCommand(
                intakeSequence,
                new ConditionalCommand(
                    getShooterReady,
                    new ConditionalCommand(
                        FeedGetShooterReady,
<<<<<<< HEAD
                        getAmpReady,
=======
                        new ConditionalCommand(
                            ShootAmp,
                            MannuleGetShooterReady,
                            () -> (Goals.getGoalInfo().goal == Goal.AMP)),
>>>>>>> parent of ffcc00d (Revert "yas")
                        () -> (Goals.getGoalInfo().goal == Goal.FEED)),
                    () -> (Goals.getGoalInfo().goal == Goal.SPEEKER)),
                () -> (Goals.getGoalInfo().goal == Goal.INTAKE)))
        .whileFalse(stopShooter);

    Contruller.getShoot()
        .whileTrue(
            new ConditionalCommand(ShootAmp, Shoot, () -> (Goals.getGoalInfo().goal == Goal.AMP)))
        .whileFalse(stopShooter);

    Contruller.ReverseIntake().whileTrue(RevIntake); // .whileFalse(stopShooter);

    Contruller.setAutoRotateOff().onTrue(Commands.runOnce(() -> Goals.setAutoRotate(false)));

    Contruller.setAutoRotateOn().onTrue(Commands.runOnce(() -> Goals.setAutoRotate(true)));

    Contruller.setDriveModeFast()
        .onTrue(Commands.runOnce(() -> Goals.setDriveMode(DriveMode.FULL)));

    Contruller.setDriveModeNormal()
        .onTrue(Commands.runOnce(() -> Goals.setDriveMode(DriveMode.NORMAL)));

    Contruller.setDriveModeSlow()
        .onTrue(Commands.runOnce(() -> Goals.setDriveMode(DriveMode.SLOW)));

    Contruller.setModeAmp().onTrue(Commands.runOnce(() -> Goals.ChangeGoal(Goal.AMP)));

    Contruller.setModeFeed().onTrue(Commands.runOnce(() -> Goals.ChangeGoal(Goal.FEED)));

    Contruller.setModeNote().onTrue(Commands.runOnce(() -> Goals.ChangeGoal(Goal.INTAKE)));

    Contruller.setModeSpeeker().onTrue(Commands.runOnce(() -> Goals.ChangeGoal(Goal.SPEEKER)));
<<<<<<< HEAD
=======

    // Contruller.setGoalMannule().onTrue(Commands.runOnce(() -> Goals.ChangeGoal(Goal.MANULE)));
>>>>>>> parent of ffcc00d (Revert "yas")

    Contruller.setPassiveSwitchOff()
        .onTrue(Commands.runOnce(() -> Goals.setPassivlysSwitch(false)));

    Contruller.setPassiveSwitchOn().onTrue(Commands.runOnce(() -> Goals.setPassivlysSwitch(true)));

    intake.NoteSensor().onTrue(Commands.runOnce(() -> CANdle.pickedUpNote()));

    // test.a().whileTrue(new AngleShooter(aim, () -> new Rotation2d(0)));

    /*
    test.a().whileTrue(intakeSequence); // .onFalse(Commands.run(() -> intakeSequence.cancel());

    OpTest.start().onTrue(Commands.run(() -> Goals.setAutoRotate(false)));
    OpTest.back().onTrue(Commands.run(() -> Goals.setAutoRotate(true)));

    test.leftTrigger().whileTrue(getShooterReady);

    test.rightTrigger()
        .whileTrue(
            new RunShooter(shooter, ShooterConstants.SpeekerShooterSpeed)
                .alongWith(new RunIntake(intake, 1)))
        .whileFalse(new RunShooter(shooter, 0).alongWith(new RunIntake(intake, 0)));

    test.y().whileTrue(getAmpReady);

    test.x().whileTrue(ShootAmp);

    test.start().onTrue(setPoseAtSpeeker);

    SmartDashboard.putData("set Pose Speeker", setPoseAtSpeeker);
    SmartDashboard.putData(getShooterReady);
    SmartDashboard.putString("Side", Math977.isRed() ? "RED" : "BLUE"); */
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
