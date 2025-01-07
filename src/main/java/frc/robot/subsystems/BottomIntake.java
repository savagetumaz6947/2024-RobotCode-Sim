package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomIntake extends SubsystemBase {
    private SparkMax intakeU = new SparkMax(55, MotorType.kBrushless);
    private SparkMax intakeD = new SparkMax(56, MotorType.kBrushless);
    private SparkMaxConfig intakeDConfig = new SparkMaxConfig();

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
    }

    public Command stop() {
        return new InstantCommand(() -> {
            rawMove(0);
        }, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("See Note?", camera.hasTargets());
    }
}
