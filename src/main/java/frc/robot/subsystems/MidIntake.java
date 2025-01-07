package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MidIntake extends SubsystemBase {    
    private SparkMax motor = new SparkMax(57, MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();

    private ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kOnboard);
    
    public MidIntake () {
        config.idleMode(IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void rawMove(double speed) {
        motor.set(speed);
    }

    public boolean hasNote() {
        return getColor().red > 0.32;
    }

    private Color getColor() {
        return sensor.getColor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Color", getColor().toString());
        SmartDashboard.putBoolean("HasNote", hasNote());
    }
}
