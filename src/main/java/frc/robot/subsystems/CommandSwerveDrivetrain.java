package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final Field2d field = new Field2d();

    private Vision s_Vision;
    private RobotConfig ppRobotConfig;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        try{
            ppRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        s_Vision = new Vision(Constants.Vision.cameraName, Constants.Vision.robotToCam, Constants.Vision.fieldLayout);
        AutoBuilder.configure(() -> this.getState().Pose, this::resetPose,
                        () -> this.getState().Speeds,
                        (speeds, feedforwards) -> this.applyRequest(speeds).execute(),
                        Constants.autoConstants,
                        ppRobotConfig,
                        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                        this);
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
                field.getObject("path").setPoses(activePath);
            });
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        try{
            ppRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        s_Vision = new Vision(Constants.Vision.cameraName, Constants.Vision.robotToCam, Constants.Vision.fieldLayout);
        AutoBuilder.configure(() -> this.getState().Pose, this::resetPose,
            () -> this.getState().Speeds,
            (speeds, feedforwards) -> this.applyRequest(speeds),
            Constants.autoConstants,
            ppRobotConfig,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this);
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
                field.getObject("path").setPoses(activePath);
            });    
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        try{
            ppRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        s_Vision = new Vision(Constants.Vision.cameraName, Constants.Vision.robotToCam, Constants.Vision.fieldLayout);
        AutoBuilder.configure(() -> this.getState().Pose, this::resetPose,
            () -> this.getState().Speeds,
            (speeds, feedforwards) -> this.applyRequest(speeds),
            Constants.autoConstants,
            ppRobotConfig,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this);
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
                field.getObject("path").setPoses(activePath);
            });
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Returns a command that drives with the default constraints with ChassisSpeeds.
     * @param speeds ChassisSpeeds
     * @return Command to run
     */
    public Command applyRequest(ChassisSpeeds speeds) {
        return this.applyRequest(() ->
            drive.withVelocityX(speeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
                .withVelocityY(speeds.vyMetersPerSecond) // Drive left with negative X (left)
                .withRotationalRate(speeds.omegaRadiansPerSecond) // Drive counterclockwise with negative X (left)
        );
    }

    /**
     * This function returns the shortest distance in 2D between the robot and the speaker of the robot's alliance.
     * @return The distance in 2D between the pose and the speaker
     */
    public double getDistToSpeaker() {
        Pose2d pose = this.getState().Pose;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
            return Math.sqrt(Math.pow(pose.getX() - Constants.GameObjects.BlueAlliance.speaker.getX(),2) + 
                                Math.pow(pose.getY() - Constants.GameObjects.BlueAlliance.speaker.getY(), 2));
        else
            return Math.sqrt(Math.pow(pose.getX() - Constants.GameObjects.RedAlliance.speaker.getX(),2) + 
                                Math.pow(pose.getY() - Constants.GameObjects.RedAlliance.speaker.getY(), 2));
    }

    public double getAngleToSpeaker() {
        Pose2d pose = this.getState().Pose;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
            return Math.atan((pose.getY() - Constants.GameObjects.BlueAlliance.speaker.getY()) / 
                                (pose.getX() - Constants.GameObjects.BlueAlliance.speaker.getX()));
        else
            return (Math.PI + Math.atan((pose.getY() - Constants.GameObjects.RedAlliance.speaker.getY()) / 
                                (pose.getX() - Constants.GameObjects.RedAlliance.speaker.getX())) + (Math.PI * 2)) % (Math.PI * 2);
    }


    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        Optional<EstimatedRobotPose> visionPose = s_Vision.getEstimatedGlobalPose(this.getState().Pose);
        if (visionPose.isPresent()) {
            this.addVisionMeasurement(visionPose.get().estimatedPose.toPose2d(), visionPose.get().timestampSeconds);
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
