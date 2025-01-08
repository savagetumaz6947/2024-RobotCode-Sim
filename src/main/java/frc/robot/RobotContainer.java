package frc.robot;

import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.logging.Logger;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;
import frc.lib.util.DeadzoneJoystick;
import frc.robot.commands.Angle.AutoRiseToAngle;
import frc.robot.commands.Angle.RiseToAngle;
import frc.robot.commands.Intake.AutoAimNote;
import frc.robot.commands.Shooting.AutoAimToShoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AngleSys;
import frc.robot.subsystems.BottomIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeAngle;
import frc.robot.subsystems.LedStrip;
import frc.robot.subsystems.MidIntake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final DeadzoneJoystick driver = new DeadzoneJoystick(0);
    private final DeadzoneJoystick operator = new DeadzoneJoystick(1);

    /* Driver Buttons */
    private final Trigger autoPickupButton = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);
    private final JoystickButton autoShootButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final Trigger autoDriveToAmpPosBtn = new Trigger(() -> {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return driver.getRawButton(XboxController.Button.kX.value);
        } else {
            return driver.getRawButton(XboxController.Button.kB.value);
        }
    });
    private final JoystickButton autoDriveToMidPosBtn = new JoystickButton(driver, XboxController.Button.kA.value);
    private final Trigger autoDriveToStagePosBtn = new Trigger(() -> {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return driver.getRawButton(XboxController.Button.kB.value);
        } else {
            return driver.getRawButton(XboxController.Button.kX.value);
        }
    });
    private final JoystickButton autoDriveToSourceBtn = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driverCancelSwerveBtn = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton fodButton = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final Trigger maxSpeedUp = new Trigger(() -> driver.getPOV() == 0);
    private final Trigger maxSpeedDown = new Trigger(() -> driver.getPOV() == 180);
    private final Trigger autoAmpBtn = new Trigger(() -> driver.getPOV() == 270);
    private final Trigger turboBtn = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.8);
    private final Trigger passBallBtn = new Trigger(() -> driver.getPOV() == 90);

    /* Operator Controls */
    private final DoubleSupplier leftClimbAxis = () -> operator.getRawAxis(XboxController.Axis.kLeftY.value);
    private final DoubleSupplier rightClimbAxis = () -> operator.getRawAxis(XboxController.Axis.kRightY.value);
    private final DoubleSupplier bottomIntakeAxis = () -> -operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) + operator.getRawAxis(XboxController.Axis.kRightTrigger.value);

    /* Operator Buttons */
    private final Trigger manualAngleUpBtn = new Trigger(() -> (operator.getPOV() == 0));
    private final Trigger manualAngleDownBtn = new Trigger(() -> (operator.getPOV() == 180));
    private final Trigger manualMidIntakeUpBtn = new Trigger(() -> (operator.getPOV() == 90));
    private final Trigger manualMidIntakeDownBtn = new Trigger(() -> (operator.getPOV() == 270));
    private final JoystickButton resetAngleBtn = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton compositeKillBtn = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton manualPickupBtn = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton forceBtn = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final Trigger trap1Btn = new Trigger(() -> {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return operator.getRawButton(XboxController.Button.kX.value);
        } else {
            return operator.getRawButton(XboxController.Button.kB.value);
        }
    });
    private final JoystickButton trap2Btn = new JoystickButton(operator, XboxController.Button.kY.value);
    private final Trigger trap3Btn = new Trigger(() -> {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return operator.getRawButton(XboxController.Button.kB.value);
        } else {
            return operator.getRawButton(XboxController.Button.kX.value);
        }
    });
    private final JoystickButton manualStartShooterBtn = new JoystickButton(operator, XboxController.Button.kA.value);

    private int maxSpeedMode = 1;

    /* Subsystems */
    private final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
    private final MidIntake midIntake = new MidIntake();
    private final BottomIntake bottomIntake = new BottomIntake();
    private final Shooter shooter = new Shooter(swerve);
    private final AngleSys angleSys = new AngleSys();
    private final Climber climber = new Climber();
    private final IntakeAngle intakeAngle = new IntakeAngle(angleSys::getDownLimit);
    private final LedStrip ledStrip = new LedStrip();

    private final RiseToAngle resetAngle = new RiseToAngle(() -> 30, angleSys);
    private final RiseToAngle riseToTrap1Angle = new RiseToAngle(() -> 49.5, angleSys);
    private final RiseToAngle riseToTrap2Angle = new RiseToAngle(() -> 49.5, angleSys);
    private final RiseToAngle riseToTrap3Angle = new RiseToAngle(() -> 49.5, angleSys);
    private final RiseToAngle riseToPassAngle = new RiseToAngle(() -> 49.5, angleSys);
    private final RiseToAngle riseToAmpAngle = new RiseToAngle(() -> 42, angleSys);

    /* Command Definitions */
    private final AutoAimToShoot autoAimToShootCommand = new AutoAimToShoot(swerve);
    private final AutoRiseToAngle autoRiseToAngleCommand = new AutoRiseToAngle(angleSys, swerve);

    private final Command pickUpNoteCommand = new InstantCommand(() -> {
            midIntake.rawMove(-1);
            bottomIntake.rawMove(0.5);
        }, midIntake, bottomIntake).repeatedly().until(() -> midIntake.hasNote()).finallyDo(() -> {
            bottomIntake.rawMove(0);
            midIntake.rawMove(0);
        });
    private final Command autoIntakeCommand = new SequentialCommandGroup(
            new AutoAimNote(swerve, bottomIntake.getCamera()),
            new ParallelDeadlineGroup(
                /* PICK UP NOTE COMMAND */
                new InstantCommand(() -> {
                    midIntake.rawMove(-1);
                    bottomIntake.rawMove(0.5);
                }, midIntake, bottomIntake).repeatedly().until(() -> midIntake.hasNote()).finallyDo(() -> {
                    bottomIntake.rawMove(0);
                    midIntake.rawMove(0);
                })
                /* PICK UP NOTE COMMAND END */,
                swerve.run(() -> swerve.applyRequest(new ChassisSpeeds(1.5, 0, 0)))
            ),
            swerve.runOnce(() -> swerve.applyRequest(new ChassisSpeeds(0, 0, 0)))
        );
    private final Command autoAmpCommand =  new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> riseToAmpAngle.isFinished()).withTimeout(2),
                new WaitUntilCommand(() -> shooter.rpmOkForAmp()).withTimeout(2),
                midIntake.run(() -> midIntake.rawMove(-1)).withTimeout(1)
            ),
            riseToAmpAngle,
            shooter.shootRepeatedlyForAmp(),
            new InstantCommand(() -> ledStrip.shoot(true))
        ).finallyDo(() -> {
            shooter.idle();
            midIntake.rawMove(0);
            ledStrip.shoot(false);
        });
    private final Command autoShootCommand = new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> autoRiseToAngleCommand.isFinished()).withTimeout(2),
                new WaitUntilCommand(() -> shooter.rpmOkForSpeaker()).withTimeout(2),
                new WaitUntilCommand(() -> autoAimToShootCommand.isFinished()).withTimeout(2),
                midIntake.run(() -> midIntake.rawMove(-1)).withTimeout(1)
            ),
            autoAimToShootCommand.andThen(swerve.run(() -> swerve.applyRequest(new ChassisSpeeds(0,0,0)))),
            autoRiseToAngleCommand,
            shooter.shootRepeatedly(),
            new InstantCommand(() -> ledStrip.shoot(true))
        ).finallyDo(() -> {
            shooter.idle();
            midIntake.rawMove(0);
            ledStrip.shoot(false);
        });

    private final SendableChooser<Command> autoChooser;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Swerve.maxSpeed * 0.1).withRotationalDeadband(Constants.Swerve.maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(Constants.Swerve.maxSpeed);

    private final SwerveRequest.Idle swerveIdle = new SwerveRequest.Idle();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Set default commands
        swerve.setDefaultCommand(
            swerve.applyRequest(() ->
                drive.withVelocityX(-driver.getRawAxis(XboxController.Axis.kLeftY.value) * Constants.Swerve.maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getRawAxis(XboxController.Axis.kLeftX.value) * Constants.Swerve.maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRawAxis(XboxController.Axis.kRightX.value) * Constants.Swerve.maxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        fodButton.onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));

        shooter.setDefaultCommand(shooter.idle());
        angleSys.setDefaultCommand(resetAngle.repeatedly());
        climber.setDefaultCommand(climber.run(() -> climber.move(leftClimbAxis, rightClimbAxis, forceBtn)));
        intakeAngle.setDefaultCommand(intakeAngle.run(() -> intakeAngle.rawMove(bottomIntakeAxis.getAsDouble() * 0.5)));

        // Register named commands
        NamedCommands.registerCommand("PickUpNote", pickUpNoteCommand);
        NamedCommands.registerCommand("Shoot", autoShootCommand);
        NamedCommands.registerCommand("DropIntake", intakeAngle.drop(angleSys));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

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
        /* Driver Buttons */
        driverCancelSwerveBtn.onTrue(new InstantCommand(() -> {
            swerve.applyRequest(() -> swerveIdle).execute();
            midIntake.rawMove(0);
            bottomIntake.rawMove(0);
            shooter.stop();
            angleSys.move(0);
            climber.move(() -> 0, () -> 0, () -> true);
        }, swerve, midIntake, bottomIntake, shooter, angleSys, climber));
        autoPickupButton.whileTrue(pickUpNoteCommand);
        autoShootButton.onTrue(autoShootCommand);
        try {
            autoDriveToAmpPosBtn.onTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ToAmpShootSpot"), Constants.defaultPathConstraints));
            autoDriveToMidPosBtn.onTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ToMidShootSpot"), Constants.defaultPathConstraints));
            autoDriveToStagePosBtn.onTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ToStageShootSpot"), Constants.defaultPathConstraints));
            autoDriveToSourceBtn.onTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ToSource"), Constants.defaultPathConstraints));
        } catch (IOException exception) {
            System.out.println("[ERROR] IOException. Does this file exist? " + exception.toString());
        } catch (ParseException exception) {
            System.out.println("[ERROR] ParseException. Did you use PathPlanner? " + exception.toString());
        }
        maxSpeedUp.onTrue(new InstantCommand(() ->
            maxSpeedMode = maxSpeedMode + 1 < Constants.Swerve.speedSelection.length ? maxSpeedMode + 1 : maxSpeedMode
        ));
        maxSpeedDown.onTrue(new InstantCommand(() ->
            maxSpeedMode = maxSpeedMode - 1 >= 0 ? maxSpeedMode - 1 : maxSpeedMode
        ));
        turboBtn.onTrue(new InstantCommand(() -> {
            maxSpeedMode = maxSpeedMode + 1 < Constants.Swerve.speedSelection.length ? maxSpeedMode + 1 : maxSpeedMode;
        }));
        turboBtn.onFalse(new InstantCommand(() -> {
            maxSpeedMode = maxSpeedMode - 1 >= 0 ? maxSpeedMode - 1 : maxSpeedMode;
        }));
        passBallBtn.onTrue(new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> riseToPassAngle.isFinished()).withTimeout(2),
                new WaitUntilCommand(() -> shooter.rpmOkForSpeaker()).withTimeout(2),
                midIntake.run(() -> midIntake.rawMove(-1)).withTimeout(1)
            ),
            riseToPassAngle,
            shooter.shootRepeatedly(),
            new InstantCommand(() -> ledStrip.shoot(true))
        ).finallyDo(() -> {
            shooter.idle();
            midIntake.rawMove(0);
            ledStrip.shoot(false);
        }));

        /* Operator Buttons */
        manualAngleUpBtn.whileTrue(angleSys.run(() -> angleSys.move(0.5)).repeatedly().finallyDo(() -> angleSys.move(0)));
        manualAngleDownBtn.whileTrue(angleSys.run(() -> angleSys.move(-0.5)).repeatedly().finallyDo(() -> angleSys.move(0)));
        manualMidIntakeUpBtn.whileTrue(midIntake.run(() -> midIntake.rawMove(-1)).repeatedly().finallyDo(() -> midIntake.rawMove(0)));
        manualMidIntakeDownBtn.whileTrue(midIntake.run(() -> midIntake.rawMove(1)).repeatedly().finallyDo(() -> midIntake.rawMove(0)));
        resetAngleBtn.onTrue(new InstantCommand(() -> angleSys.reset()));
        compositeKillBtn.whileTrue(new InstantCommand(() -> {
            swerve.applyRequest(new ChassisSpeeds());
            midIntake.rawMove(0);
            bottomIntake.rawMove(0);
            shooter.stop();
            angleSys.move(0);
            climber.move(() -> 0, () -> 0, () -> true);
        }, swerve, midIntake, bottomIntake, shooter, angleSys, climber));
        manualPickupBtn.onTrue(autoIntakeCommand);
        try {
            trap1Btn.onTrue(new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("ToTrap1"), Constants.defaultPathConstraints),
                    new WaitUntilCommand(() -> riseToTrap1Angle.isFinished()).withTimeout(2),
                    midIntake.run(() -> midIntake.rawMove(-1)).withTimeout(1)
                ),
                riseToTrap1Angle,
                shooter.shootRepeatedly()
            ).finallyDo(() -> midIntake.rawMove(0)));
            trap2Btn.onTrue(new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("ToTrap2"), Constants.defaultPathConstraints),
                    new WaitUntilCommand(() -> riseToTrap2Angle.isFinished()).withTimeout(2),
                    midIntake.run(() -> midIntake.rawMove(-1)).withTimeout(1)
                ),
                riseToTrap2Angle,
                shooter.shootRepeatedly()
            ).finallyDo(() -> midIntake.rawMove(0)));
            trap3Btn.onTrue(new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("ToTrap3"), Constants.defaultPathConstraints),
                    new WaitUntilCommand(() -> riseToTrap3Angle.isFinished()).withTimeout(2),
                    midIntake.run(() -> midIntake.rawMove(-1)).withTimeout(1)
                ),
                riseToTrap3Angle,
                shooter.shootRepeatedly()
            ).finallyDo(() -> midIntake.rawMove(0)));
        } catch (IOException exception) {
            System.out.println("[ERROR] IOException. Does this file exist? " + exception.toString());
        } catch (ParseException exception) {
            System.out.println("[ERROR] ParseException. Did you use PathPlanner? " + exception.toString());
        }
        manualStartShooterBtn.whileTrue(shooter.run(() -> shooter.reverse()).andThen(shooter.idle()));
        autoAmpBtn.onTrue(autoAmpCommand);

        swerve.registerTelemetry(logger::telemeterize);
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
