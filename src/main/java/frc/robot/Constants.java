package frc.robot;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    // Inital pose of the robot
    public static Pose2d initialPose = new Pose2d(1.24, 5.57, new Rotation2d(0));

    public static final class Vision {
        public static final String cameraName = "BackCamera";
        // Cam mounted facing foward, 54 degrees up, at the front of the robot, 23cm up.
        public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.02-0.36,0,0.52), new Rotation3d(0, Math.toRadians(58-90), Math.PI));
        // public static final Transform3d robotToCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0, 0, 0));

        // public static final AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(List.of(
        //     new AprilTag(1, new Pose3d(1.5, 2, 0.5, new Rotation3d(0, 0, Math.PI)))
        // ), 16.4846, 8.1026);

        public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    }

    public static final class GameObjects {
        public static final class BlueAlliance {
            public static final Pose3d speaker = new Pose3d(0, 5.54, 2.05, new Rotation3d());
        }
        public static final class RedAlliance {
            public static final Pose3d speaker = new Pose3d(16.54, 5.54, 2.05, new Rotation3d(Math.PI, 0, 0));
        }
    }

    public static final class Swerve {
        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double[] speedSelection = {3.0, 4.5, 5.5}; //TODO: You can set this to your desired speed

        public static final double teleopMaxWheelAcceleration = 15;
        /** Radians per Second */
        public static final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        public static final double teleopMaxAngularAcceleration = Units.degreesToRadians(720);
    }

    public static final PPHolonomicDriveController autoConstants = new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(1.9, 0, 0), // Translation PID constants
        new PIDConstants(3, 0.0, 0.0) // Rotation PID constants
    );

    public static final PathConstraints defaultPathConstraints = new PathConstraints(4.5, 5, Math.toRadians(420), Math.toRadians(720));
}
