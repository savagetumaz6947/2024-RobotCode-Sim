package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;

import org.ironmaple.simulation.IntakeSimulation;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class BottomIntake extends SubsystemBase {
    private SparkMax intakeU = new SparkMax(55, MotorType.kBrushless);
    private SparkMax intakeD = new SparkMax(56, MotorType.kBrushless);
    private SparkMaxConfig intakeDConfig = new SparkMaxConfig();

    private static FlywheelSim sim;
    private SparkMaxSim intakeUSim;
    private SparkMaxSim intakeDSim;

    public static IntakeSimulation intakeSim;

    private final Vision camera = new Vision("IntakeCamera");

    public BottomIntake() {
        intakeDConfig.follow(intakeU);
        intakeD.configure(intakeDConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Vision getCamera() {
        return camera;
    }

    public void rawMove (double speed) {
        intakeU.set(-speed);
        if (Robot.isSimulation()) {
            if (speed > 0 && IntakeAngle.sim.hasHitLowerLimit())
                intakeSim.startIntake();
            else
                intakeSim.stopIntake();
        }
    }

    public Command stop() {
        return new InstantCommand(() -> {
            rawMove(0);
        }, this);
    }

    public void configureSimulation() {
        sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, 1), DCMotor.getNEO(1));

        intakeUSim = new SparkMaxSim(intakeU, DCMotor.getNEO(1));
        intakeDSim = new SparkMaxSim(intakeD, DCMotor.getNEO(1));

        intakeSim = IntakeSimulation.OverTheBumperIntake("Note", 
            CommandSwerveDrivetrain.mapleSimSwerveDrivetrain.mapleSimDrive,
            Millimeters.of(515.4),
            Millimeters.of(150),
            IntakeSimulation.IntakeSide.FRONT,
            1);
    }

    @Override
    public void simulationPeriodic() {
        sim.setInputVoltage(intakeUSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        sim.update(0.02);

        intakeUSim.iterate(Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec()), RobotController.getBatteryVoltage(), 0.02);
        intakeDSim.iterate(Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec()), RobotController.getBatteryVoltage(), 0.02);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("See Note?", camera.hasTargets());
    }
}
