package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private SparkMax up = new SparkMax(62, MotorType.kBrushless);
    private SparkMax down = new SparkMax(61, MotorType.kBrushless);

    private CommandSwerveDrivetrain swerve;

    public static FlywheelSim sim;
    private SparkMaxSim upSim;
    private SparkMaxSim downSim;

    public Shooter(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
    }

    public Command shootRepeatedly() {
        return this.runEnd(() -> {
            up.set(-1);
            down.set(1);
        }, () -> up.set(0));
    }

    public Command shootRepeatedlyForAmp() {
        return this.runEnd(() -> {
            up.set(-0.7);
            down.set(.7);
        }, () -> up.set(0));
    }
    // Amp 40.3 DEG

    public boolean inRange() {
        return swerve.getDistToSpeaker() < 3.5;
    }

    public Command idle() {
        return new InstantCommand(() -> {
            if (DriverStation.isAutonomous()) {
                up.set(-1);
                down.set(1);
            } else if (inRange()) {
                up.set(-0.2);
                down.set(0.2);
            } else {
                up.set(0);
                down.set(0);
            }
        }, this).repeatedly().finallyDo(() -> {
            up.set(0);
            down.set(0);
        });
    }

    public Command revIdle() {
        return this.runEnd(() -> {
            up.set(0.3);
            down.set(-0.3);
        }, () -> {
            up.set(0);
            down.set(0);
        });
    }

    public void reverse() {
        up.set(0.5);
        down.set(-0.5);
    }

    public void stop() {
        up.set(0);
        down.set(0);
    }

    public boolean rpmOkForSpeaker() {
        return up.getEncoder().getVelocity() <= -4800;
    }

    public boolean rpmOkForAmp() {
        return up.getEncoder().getVelocity() <= -2400;
    }

    public void configureSimulation() {
        sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, 3), DCMotor.getNEO(1));

        upSim = new SparkMaxSim(up, DCMotor.getNEO(1));
        downSim = new SparkMaxSim(down, DCMotor.getNEO(1));
    }

    @Override
    public void simulationPeriodic() {
        sim.setInputVoltage(upSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        sim.update(0.02);

        upSim.iterate(Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec() * 3), RobotController.getBatteryVoltage(), 0.02);
        downSim.iterate(Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec() * 3), RobotController.getBatteryVoltage(), 0.02);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/UP_RPM",  up.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/DOWN_RPM",  up.getEncoder().getVelocity());

        SmartDashboard.putBoolean("Shooter In Range", inRange());
    }
}
