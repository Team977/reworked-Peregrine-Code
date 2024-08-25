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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.TeleoperatedAutos.ModeController;
import frc.robot.TeleoperatedAutos.ModeController.RobotGoal;
import frc.robot.TeleoperatedAutos.RotationAutoPilot;
import frc.robot.TeleoperatedAutos.ShooterAutoPilot;
import frc.robot.commandGroups.IntakeSequence;
import frc.robot.commandGroups.NewGetAmpReady;
import frc.robot.commandGroups.ResetShooter;
import frc.robot.commandGroups.Shoot;
import frc.robot.commandGroups.getShooterReady;
import frc.robot.commandGroups.getShooterReadyManual;
import frc.robot.commandGroups.getShooterReadyNoPrep;
import frc.robot.commands.AngleShooter;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToNote;
import frc.robot.commands.FireNoteAmp;
import frc.robot.commands.LoadMag;
import frc.robot.commands.ReverseIntake;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Aim;
import frc.robot.subsystems.shooter.MagRoller;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterEjectSpeeds;

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
  private final Intake intake;
  private final MagRoller magRoller;
  private final Shooter shooter;
  private final Aim aim;
  private final Elevator elevator;
  private static final Candle candle = new Candle();
  public static final Vision vision = new Vision();

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  public static final ModeController modeController = new ModeController();
  public static final RotationAutoPilot rotationAutoPilot = new RotationAutoPilot();
  public static final ShooterAutoPilot shooterAutoPilot = new ShooterAutoPilot();

  //  private final OIbase OI = new OIGuitarAndJoystick(1,0);
  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOSparkMax(0),
            new ModuleIOSparkMax(1),
            new ModuleIOSparkMax(2),
            new ModuleIOSparkMax(3));
    intake = new Intake();
    elevator = new Elevator();
    shooter = new Shooter();
    aim = new Aim();
    magRoller = new MagRoller();

    // Set up auto routines

    NamedCommands.registerCommand("FireNote", new Shoot(shooter, magRoller).withTimeout(0.3));

    NamedCommands.registerCommand(
        "ResetShooter", new ResetShooter(elevator, aim, shooter).withTimeout(0.75));

    NamedCommands.registerCommand(
        "RunIntake", new IntakeSequence(intake, magRoller).withTimeout(4.0));

    NamedCommands.registerCommand(
        "RunMagAndIntake", new LoadMag(intake, magRoller).withTimeout(0.1));

    NamedCommands.registerCommand(
        "GetShooterReady", new getShooterReady(aim, elevator, magRoller, shooter, 5.0));

    NamedCommands.registerCommand(
        "aim", new AngleShooter(aim, () -> ShooterAutoPilot.GetShooterAngleToSpeaker()));

    NamedCommands.registerCommand(
        "GetShooterReadyClose",
        new getShooterReadyNoPrep(aim, elevator, magRoller, shooter, 5.0).withTimeout(0.4));

    NamedCommands.registerCommand(
        "GetShooterReadyCloseFast",
        new getShooterReadyNoPrep(aim, elevator, magRoller, shooter, 5.0).withTimeout(0.8));

    NamedCommands.registerCommand(
        "GoToNote",
        new DriveToNote(
                drive, () -> intake.isNotePresent(), () -> drive.getRotation().getRadians(), 2)
            .raceWith(new IntakeSequence(intake, magRoller)));

    NamedCommands.registerCommand(
        "getShooterreadyfar",
        new getShooterReadyManual(
                aim,
                elevator,
                magRoller,
                shooter,
                0.5,
                () -> -65, // -35
                ShooterEjectSpeeds.Medium)
            .withTimeout(1.75));

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

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftX(),
            () -> -driverController.getLeftY(),
            () -> -driverController.getRightX()));

    aDirection = SysIdRoutine.Direction.kForward;
    bDirection = SysIdRoutine.Direction.kReverse;

    driverController
        .a()
        .whileTrue(
            Commands.run(() -> elevator.setPosition(3), elevator)
                .withTimeout(2)
                .andThen(elevator.sysIdQuasistatic(aDirection).withTimeout(2.5))
                .andThen(elevator.sysIdQuasistatic(bDirection).withTimeout(2.5))
                .andThen(elevator.sysIdDynamic(aDirection).withTimeout(.6))
                .andThen(elevator.sysIdDynamic(bDirection).withTimeout(.45)));

    // driverController.a().whileTrue(elevator.sysIdQuasistatic(aDirection));
    driverController.b().whileTrue(elevator.sysIdQuasistatic(bDirection));

    driverController.y().whileTrue(elevator.sysIdDynamic(aDirection));
    driverController.x().whileTrue(elevator.sysIdDynamic(bDirection));

    // driverController.x().whileTrue(new ElevatorMoveToPosition(elevator, elevatorPos.Amp));
    // driverController.y().whileTrue(new ElevatorMoveToPosition(elevator, elevatorPos.Normal));

    // configs drive
    // Driver Controls

    // may want to remove this, make it operator-only
    /*driverController
    .leftTrigger()
    .whileTrue(
        new ConditionalCommand(
            new getShooterReady(aim, elevator, magRoller, shooter, 5.0),
            new GetAmpReady(elevator, aim),
            () -> ModeController.getRobotGoal() == RobotGoal.SPEAKER));*/
    /*
    driverController
        .rightTrigger()
        .whileTrue(
            new ConditionalCommand(

                // shooter and amp
                new ConditionalCommand(
                    // shooter
                    new Shoot(shooter, magRoller),
                    // amp
                    new FireNoteAmp(shooter, magRoller),
                    () ->
                        (modeController.getRobotGoal() == RobotGoal.SPEAKER
                            || modeController.getRobotGoal() == RobotGoal.SPEAKER_MAUNLUE))

                // source and ground
                ,
                new IntakeSequence(intake, magRoller),
                () ->
                    (modeController.getRobotGoal() == ModeController.RobotGoal.AMP
                        || modeController.getRobotGoal() == RobotGoal.SPEAKER
                        || modeController.getRobotGoal() == RobotGoal.SPEAKER_MAUNLUE)));

                       */
    driverController
        .rightTrigger()
        .onTrue(
            new ConditionalCommand(
                new Shoot(shooter, magRoller),
                new FireNoteAmp(shooter, magRoller),
                () ->
                    modeController.getRobotGoal() == RobotGoal.SPEAKER
                        || modeController.getRobotGoal() == RobotGoal.FEED
                        || modeController.getRobotGoal() == RobotGoal.SPEAKER_MANUAL))
        .onFalse(
            new ResetShooter(elevator, aim, shooter)
                .alongWith(Commands.runOnce(modeController::setGoalGroundNote)));

    // drive mode
    driverController
        .rightBumper()
        .whileTrue(Commands.runOnce(modeController::setDriveModePrecision))
        .onFalse(Commands.runOnce(modeController::setDriveModeNormal));

    driverController
        .leftBumper()
        .whileTrue(Commands.runOnce(modeController::setDriveModeSpeedy))
        .onFalse(Commands.runOnce(modeController::setDriveModeNormal));

    // driverController.leftBumper().whileTrue(new IntakeSequence(intake, magRoller));

    driverController.start().onTrue(Commands.runOnce(modeController::setAutoRotateModeOff));

    // driverController.a().whileTrue(Commands.run(drive::stopWithX, drive));

    // driverController.y().whileTrue(new ClimbAndTrap(elevator, aim, shooter, magRoller));

    //    driverController
    //       .b()
    //      .whileTrue(
    //          new DriveToNote(
    //                drive, () -> intake.isNotePresent(), () -> drive.getRotation().getRadians(),
    // 3)
    //          .raceWith(new IntakeSequence(intake, magRoller)));

    // Operator Controls

    // operatorController.leftTrigger().onTrue(Commands.runOnce(modeController::setAutoRotateModeOn));
    operatorController.start().onTrue(Commands.runOnce(modeController::setAutoRotateModeOff));
    /*
    operatorController
        .leftTrigger()
        .whileTrue(
            new ConditionalCommand(
                new getShooterReady(aim, elevator, magRoller, shooter, 5.0),
                new GetAmpReady(elevator, aim),
                () ->
                    (ModeController.getRobotGoal() == RobotGoal.SPEAKER
                        || ModeController.getRobotGoal() == RobotGoal.SPEAKER_MAUNLUE)));
             */
    operatorController
        .leftTrigger()
        .onTrue(Commands.runOnce(modeController::setAutoRotateModeOn))
        .onFalse(Commands.runOnce(modeController::setAutoRotateModeOff))
        .whileTrue(
            new ConditionalCommand(

                // shooter and amp
                new ConditionalCommand(
                    // shooter
                    new ConditionalCommand(
                        new ConditionalCommand(
                            new getShooterReady(aim, elevator, magRoller, shooter, 0.5),
                            new getShooterReadyManual(
                                aim,
                                elevator,
                                magRoller,
                                shooter,
                                0,
                                () -> -40,
                                ShooterEjectSpeeds.Slow),
                            () -> modeController.getRobotGoal() == RobotGoal.SPEAKER),
                        new getShooterReadyManual(
                            aim,
                            elevator,
                            magRoller,
                            shooter,
                            0.5,
                            () -> SmartDashboard.getNumber("aim angle", -35), // -35
                            ShooterEjectSpeeds.Medium),
                        () ->
                            modeController.getRobotGoal() == RobotGoal.SPEAKER
                                || modeController.getRobotGoal() == RobotGoal.FEED),
                    // amp
                    new NewGetAmpReady(elevator, magRoller, aim, shooter),
                    () ->
                        (modeController.getRobotGoal() == RobotGoal.SPEAKER
                            || modeController.getRobotGoal() == RobotGoal.SPEAKER_MANUAL
                            || modeController.getRobotGoal() == RobotGoal.FEED)),

                // source and ground

                new IntakeSequence(intake, magRoller),
                () ->
                    (modeController.getRobotGoal() == ModeController.RobotGoal.AMP
                        || modeController.getRobotGoal() == RobotGoal.SPEAKER
                        || modeController.getRobotGoal() == RobotGoal.SPEAKER_MANUAL
                        || modeController.getRobotGoal() == RobotGoal.FEED)));
    /*
    operatorController
        .leftTrigger()
        .whileTrue(
            //check to see if the mode is eather Speeker or Manule
            new ConditionalCommand(
                //if mode is mannule or Speeker ->
                new ConditionalCommand(
                    //speaker
                    new getShooterReadyAuto(aim, elevator, magRoller, shooter, 0.5),
                    //manual
                    new getShooterReady(aim, elevator, magRoller, shooter, 0.5),
                    //condition
                    () -> ModeController.getRobotGoal() == RobotGoal.SPEAKER),
                null,
                () ->
                    (ModeController.getRobotGoal() == RobotGoal.SPEAKER
                        || ModeController.getRobotGoal() == RobotGoal.SPEAKER_MAUNLUE)));
                        */
    operatorController.back().onTrue(Commands.runOnce(drive::resetPose));

    operatorController.leftStick().whileTrue(new ResetShooter(elevator, aim, shooter));
    operatorController.rightStick().whileTrue(new ResetShooter(elevator, aim, shooter));

    operatorController.a().whileTrue(new IntakeSequence(intake, magRoller));

    operatorController.b().whileTrue(new ReverseIntake(intake));

    operatorController.povUp().onTrue(Commands.runOnce(modeController::setManualAiming));

    operatorController.y().onTrue(Commands.runOnce(modeController::setGoalSpeaker));
    operatorController.x().onTrue(Commands.runOnce(modeController::setGoalAmp));
    operatorController.leftBumper().onTrue(Commands.runOnce(modeController::setGoalGroundNote));
    operatorController.rightBumper().onTrue(Commands.runOnce(modeController::setGoalFeed));
    // operatorController.povUp().onTrue(Commands.runOnce(modeController::setGoalClimbBack));
    operatorController.povLeft().onTrue(Commands.runOnce(modeController::setGoalClimbLeft));
    operatorController.povRight().onTrue(Commands.runOnce(modeController::setGoalClimbRight));
    /*operatorController
          .povDown()
          .onTrue(Commands.runOnce(() -> modeController.setRobotGoal(RobotGoal.OVER_TRAP)));
      // shoot With auto Aim

      driverController
          .leftBumper()
          .whileTrue(new ShootSequenceAuto(aim, magRoller, shooter, elevator))
          .onFalse(new StopShooterMotors(shooter).withTimeout(1.0));

    // shoot without auto Aim
      controller
          .rightBumper()
          .whileTrue(new ShootSequenceManual(aim, magRoller, shooter, elevator, -35.0))
          .onFalse(new StopShooterMotors(shooter).withTimeout(1.0));

        controller
            .leftBumper()
            .whileTrue(new getShooterReady(aim, elevator, magRoller, shooter, 20))
            .onFalse(new ResetShooter(elevator, aim));

        controller.rightBumper().whileFalse(new Shoot(shooter, magRoller));

        controller.b().whileTrue(new AmpScore(elevator, aim, magRoller));

        controller.y().whileTrue(new GetFromSource(elevator, aim, magRoller, shooter));

        controller.a().whileTrue(new IntakeSequence(intake, magRoller));

        controller.x().whileTrue(new ResetShooter(elevator, aim));
      */

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
