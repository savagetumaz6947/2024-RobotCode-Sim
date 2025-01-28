package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPoseEstimator photonEstimator;

    public VisionSystemSim sim;
    private PhotonCameraSim simCam;
    private SimCameraProperties simCameraProperties = new SimCameraProperties();
    private Pose2d prevPose = new Pose2d(); // this is only used for simulation

    private Optional<Supplier<Pose2d>> maplePoseSupplier = Optional.empty();

    /**
     * Constructs a Vision object with no transformation (primarily for use with Object Detection)
     * @param cameraName The name of the PhotonCamera
     */
    public Vision(String cameraName) {
        this(cameraName, new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));
    }

    /**
     * Constructs a Vision object with the predefined field layout
     * @param cameraName The name of the PhotonCamera
     * @param robotToCam The location of the camera relative to the robot center
     */
    public Vision(String cameraName, Transform3d robotToCam) {
        this(cameraName, robotToCam, AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
    }

    /**
     * Constructs a Vision object with custom AprilTagFieldLayout
     * @param cameraName The name of the PhotonCamera
     * @param robotToCam The location of the camera relative to the robot center
     * @param aprilTagFieldLayout The custom AprilTagFieldLayout
     */
    public Vision(String cameraName, Transform3d robotToCam, AprilTagFieldLayout aprilTagFieldLayout) {
        this(cameraName, robotToCam, aprilTagFieldLayout, Optional.empty());
    }

    public Vision(String cameraName, Transform3d robotToCam, AprilTagFieldLayout aprilTagFieldLayout, Optional<Supplier<Pose2d>> maplePoseSupplier) {
        camera = new PhotonCamera(cameraName);
        photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);

        if (Robot.isSimulation()) {
            sim = new VisionSystemSim(cameraName);
            sim.addAprilTags(aprilTagFieldLayout);
            simCameraProperties.setCalibration(1920, 1080, Rotation2d.fromDegrees(100));
            simCameraProperties.setCalibError(0.25, 0.08);
            simCameraProperties.setFPS(60);
            simCameraProperties.setAvgLatencyMs(35);
            simCameraProperties.setLatencyStdDevMs(5);

            simCam = new PhotonCameraSim(camera, simCameraProperties);
            sim.addCamera(simCam, robotToCam);

            if (maplePoseSupplier.isEmpty()) {
                System.out.println("[WARN] Vision.java NO MAPLE POSE SUPPLIER GIVEN. The simulation will assume its pose to be the drivetrain pose.");
            } else {
                this.maplePoseSupplier = maplePoseSupplier;
            }
        }
    }

    /**
     * Returns whether the camera pipeline has targets. Will call getLatestResult.
     * @return Whether the camera pipeline has targets
     */
    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        return camera.getLatestResult().getBestTarget();
    }

    public void setLastPose(Pose2d pose) {
        photonEstimator.setLastPose(pose);
    }

    @Override
    public void simulationPeriodic() {
        if (maplePoseSupplier.isPresent()) {
            sim.update(maplePoseSupplier.get().get());
        }
        else {
            sim.update(prevPose);
        }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     * 
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        prevPose = prevEstimatedRobotPose;

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult estimator : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(estimator);
        }
        return visionEst;
    }
}
