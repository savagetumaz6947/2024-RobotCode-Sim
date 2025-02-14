package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAngle extends SubsystemBase {
    private SparkMax motor = new SparkMax(58, MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();
    private BooleanSupplier intakeBottom;
    private DigitalInput upLimit = new DigitalInput(0);
    private DigitalInput downLimit = new DigitalInput(1);

    private DIOSim upLimitSim;
    private DIOSim downLimitSim;

    private SparkMaxSim motorSim;

    public static SingleJointedArmSim sim;

    @AutoLogOutput
    private Pose3d simPose = new Pose3d();

    public IntakeAngle (BooleanSupplier intakeBottomSupplier) {
        config.idleMode(IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.getEncoder().setPosition(0);
        intakeBottom = intakeBottomSupplier;
    }

    public void rawMove (double value) {
        if (value > 0) {
            if (!downLimit.get()) {
                motor.set(value);
            } else {
                motor.set(0);
            }
        } else {
            if (!upLimit.get() && !intakeBottom.getAsBoolean()) {
                motor.set(value);
            } else {
                motor.set(0);
            }
        }
    }

    public Command drop(AngleSys angleSys) {
        Command command = this.runOnce(() -> {
            rawMove(0.5);
        }).repeatedly().withTimeout(2).finallyDo(() -> {
            rawMove(0);
        });
        command.addRequirements(angleSys);
        return command;
    }

    public void configureSimulation() {
        motorSim = new SparkMaxSim(motor, DCMotor.getNEO(1));
        sim = new SingleJointedArmSim(DCMotor.getNEO(1), 200, 2,
            0.31261, Units.degreesToRadians(11), Units.degreesToRadians(143),
            true, Units.degreesToRadians(143));
        
        upLimitSim = new DIOSim(upLimit);
        downLimitSim = new DIOSim(downLimit);
    }

    @Override
    public void simulationPeriodic() {
        sim.setInputVoltage(-motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        sim.update(0.02);

        motorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(sim.getVelocityRadPerSec() * 200), RobotController.getBatteryVoltage(), 0.02);

        simPose = new Pose3d(
            new Translation3d(Millimeters.of(226.1605), Meters.of(0), Millimeters.of(231.691+50.8)),
            new Rotation3d(Radians.of(0), Radians.of(-sim.getAngleRads()), Radians.of(0)));
        
        upLimitSim.setValue(sim.hasHitUpperLimit());
        downLimitSim.setValue(sim.hasHitLowerLimit());
    }

    @Override
    public void periodic() {
        // Logger.recordOutput("IntakeAngle/EncoderPos", motor.getEncoder().getPosition());
        // Logger.recordOutput("IntakeAngle/DownLimit", downLimit.get());
        // Logger.recordOutput("IntakeAngle/UpLimit", upLimit.get());
        SmartDashboard.putBoolean("IntakeAngle/AtBottom", downLimit.get());
        Logger.recordOutput("IntakeAngle/motor/output", motorSim.getAppliedOutput());
    }
}
