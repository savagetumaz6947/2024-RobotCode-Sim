package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(51, "canivore");
    private TalonFX rightMotor = new TalonFX(52, "canivore");

    private static ElevatorSim leftSim;
    private static ElevatorSim rightSim;

    @AutoLogOutput
    private static Pose3d leftSimPose = new Pose3d();
    @AutoLogOutput
    private static Pose3d rightSimPose = new Pose3d();

    public Climber () {
        TalonFXConfigurator leftConfig = leftMotor.getConfigurator();
        TalonFXConfigurator rightConfig = rightMotor.getConfigurator();
        MotorOutputConfigs leftMotorOutputConfigs = new MotorOutputConfigs();
        MotorOutputConfigs rightMotorOutputConfigs = new MotorOutputConfigs();
        leftMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        rightMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        leftMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        leftConfig.apply(leftMotorOutputConfigs);
        rightConfig.apply(rightMotorOutputConfigs);

        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }

    public void move (DoubleSupplier left, DoubleSupplier right, BooleanSupplier force) {
        if (force.getAsBoolean()) {
            leftMotor.set(left.getAsDouble());
            rightMotor.set(right.getAsDouble());
        } else {
            if ((left.getAsDouble() < 0 && leftMotor.getPosition().getValueAsDouble() > -180) || (left.getAsDouble() > 0 && leftMotor.getPosition().getValueAsDouble() < 0)) {
                leftMotor.set(left.getAsDouble());
            } else {
                leftMotor.set(0);
            }

            if ((right.getAsDouble() < 0 && rightMotor.getPosition().getValueAsDouble() > -180) || (right.getAsDouble() > 0 && rightMotor.getPosition().getValueAsDouble() < 0)) {
                rightMotor.set(right.getAsDouble());
            } else {
                rightMotor.set(0);
            }
        }
    }

    public void configureSimulation() {
        leftSim = new ElevatorSim(DCMotor.getFalcon500(1), 200, 56, 0.0508,
                0, 0.35-0.03, false,
                0);
        rightSim = new ElevatorSim(DCMotor.getFalcon500(1), 200, 56, 0.0508,
                0, 0.35-0.03, false,
                0);
        rightMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    }

    public void simulationPeriodic() {
        leftMotor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        leftSim.setInputVoltage(leftMotor.getSimState().getMotorVoltage());
        leftSim.update(0.02);

        leftSimPose = new Pose3d(new Translation3d(Meters.of(0), Meters.of(0), Meters.of(leftSim.getPositionMeters())), new Rotation3d());

        leftMotor.getSimState().setRawRotorPosition(Radians.of(leftSim.getPositionMeters() / 0.0508 * 200));
        leftMotor.getSimState().setRotorVelocity(RadiansPerSecond.of(leftSim.getVelocityMetersPerSecond() / 0.0508 * 200));

        rightMotor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSim.setInputVoltage(rightMotor.getSimState().getMotorVoltage());
        rightSim.update(0.02);

        rightSimPose = new Pose3d(new Translation3d(Meters.of(0), Meters.of(0), Meters.of(rightSim.getPositionMeters())), new Rotation3d());

        rightMotor.getSimState().setRawRotorPosition(Radians.of(rightSim.getPositionMeters() / 0.0508 * 200));
        rightMotor.getSimState().setRotorVelocity(RadiansPerSecond.of(rightSim.getVelocityMetersPerSecond() / 0.0508 * 200));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Climber/Left", leftMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Climber/Right", rightMotor.getPosition().getValueAsDouble());
    }
}
