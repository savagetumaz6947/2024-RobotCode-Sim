package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class MidIntake extends SubsystemBase {    
    private SparkMax motor = new SparkMax(57, MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();

    private ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kOnboard);

    private static FlywheelSim sim;
    private SparkMaxSim motorSim;

    @AutoLogOutput(key = "FieldSimulation/NoteOnRobot")
    private Pose3d noteOnRobot = new Pose3d();
    
    public MidIntake () {
        config.idleMode(IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void rawMove(double speed) {
        motor.set(speed);
    }

    public boolean hasNote() {
        if (Robot.isReal())
            return getColor().red > 0.32;
        else
            return BottomIntake.intakeSim.getGamePiecesAmount() != 0;
    }

    private Color getColor() {
        return sensor.getColor();
    }

    public void configureSimulation() {
        sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0.01, 5), DCMotor.getNeo550(1));

        motorSim = new SparkMaxSim(motor, DCMotor.getNeo550(1));
    }

    @Override
    public void simulationPeriodic() {
        sim.setInputVoltage(motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        sim.update(0.02);

        motorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec() * 5), RobotController.getBatteryVoltage(), 0.02);

        if (hasNote()) {
            double currAngle = AngleSys.fourBarConversion(Radians.of(AngleSys.sim.getAngleRads())).plus(Degrees.of(10.3)).in(Radians);
            // These values are measured using CAD and trignometry.
            noteOnRobot = new Pose3d(CommandSwerveDrivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose()).plus(
                new Transform3d(Millimeters.of(308.198 - 173.205 * Math.cos(currAngle)), Meters.of(0), Millimeters.of(215 + 50.8 + 173.205 * Math.sin(currAngle)),
                new Rotation3d(Degrees.of(0), Radians.of(currAngle), Degrees.of(0))));
        } else {
            noteOnRobot = new Pose3d();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Color", getColor().toString());
        Logger.recordOutput("HasNote", hasNote());
        SmartDashboard.putBoolean("HasNote", hasNote());
    }
}
