package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAngle extends SubsystemBase {
    private SparkMax motor = new SparkMax(58, MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();
    private BooleanSupplier intakeBottom;
    private DigitalInput upLimit = new DigitalInput(0);
    private DigitalInput downLimit = new DigitalInput(1);

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

    @Override
    public void periodic() {
        // Logger.recordOutput("IntakeAngle/EncoderPos", motor.getEncoder().getPosition());
        // Logger.recordOutput("IntakeAngle/DownLimit", downLimit.get());
        // Logger.recordOutput("IntakeAngle/UpLimit", upLimit.get());
        SmartDashboard.putBoolean("IntakeAngle/AtBottom", downLimit.get());
    }
}
