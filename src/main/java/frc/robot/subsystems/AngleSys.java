package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SimmableDigitalInput;

public class AngleSys extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(41, "canivore");
    private TalonFX rightMotor = new TalonFX(42, "canivore");

    private SparkMax sparkMaxEncoderOnly = new SparkMax(43, MotorType.kBrushed);
    private SimmableDigitalInput downLimit = new SimmableDigitalInput(9, () -> !sim.hasHitLowerLimit());
    private SimmableDigitalInput upLimit = new SimmableDigitalInput(8, () -> !sim.hasHitUpperLimit());

    private SparkRelativeEncoder encoder;
    private SparkRelativeEncoderSim encoderSim;

    private static SingleJointedArmSim sim;

    @AutoLogOutput
    private Pose3d simPose = new Pose3d();

    public AngleSys() {
        encoder = (SparkRelativeEncoder) sparkMaxEncoderOnly.getEncoder();

        TalonFXConfigurator leftConfig = leftMotor.getConfigurator();
        TalonFXConfigurator rightConfig = rightMotor.getConfigurator();
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        leftConfig.apply(motorConfigs);
        rightConfig.apply(motorConfigs);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        encoder.setPosition(0);
    }

    @AutoLogOutput
    public double getAngle() {
        return -encoder.getPosition() * 360 + 29;
    }

    public boolean getDownLimit() {
        return downLimit.get();
    }

    /**
     * This function returns the automatically calculated angle in order for the shooter to be shot accurately into the speaker.
     * When it is not in range, it will return 34 degrees, the minimum angle available.
     * @return
     */
    public double getAutoAngle(double dist) {
        double value = (58.9 - 8.09 * dist);
        if (value < 29) return 29;
        else if (value > 60) return 60;
        return value; // Refer to Google Sheets
    }

    public void move(double val) {
        if ((downLimit.get() && val < 0) || (upLimit.get() && SmartDashboard.getBoolean("IntakeAngle/AtBottom", false) && val > 0)) {
            final DutyCycleOut lRequest = new DutyCycleOut(-val * 1);
            leftMotor.setControl(lRequest);
        } else {
            final DutyCycleOut stopRequest = new DutyCycleOut(0);
            leftMotor.setControl(stopRequest);
        }
    }

    public void reset() {
        encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AngleSys/EncoderPos", encoder.getPosition());
        SmartDashboard.putNumber("AngleSys/EncoderAngleDeg", getAngle());
    }

    private Angle fourBarConversion(Angle motorBarAngle) {
        double y = motorBarAngle.in(Radians);
        return Radians.of(-Math.asin((37*Math.sqrt(2)*(10*Math.cos(y)+29))/(60*Math.sqrt(555*Math.cos(y)+797)))+Math.atan((15*Math.sin(y))/(15*Math.cos(y)+37))+Math.PI/2);
    }

    public void configureSimulation() {
        sim = new SingleJointedArmSim(
            DCMotor.getFalcon500(2), 100, 0.2, 99999999,
            Units.degreesToRadians(-9.896), Units.degreesToRadians(104.4576547), false, Units.degreesToRadians(-9.896));
        
        leftMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        encoderSim = new SparkRelativeEncoderSim(sparkMaxEncoderOnly);
    }

    @Override
    public void simulationPeriodic() {
        leftMotor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        sim.setInputVoltage(leftMotor.getSimState().getMotorVoltage());
        sim.update(0.02);

        Logger.recordOutput("AngleSys/SimDeg", fourBarConversion(Radians.of(sim.getAngleRads())).in(Degrees));

        Angle newDeg = fourBarConversion(Radians.of(sim.getAngleRads())).plus(Degrees.of(10.3));
        simPose = new Pose3d(
            new Translation3d(Millimeters.of(308.198), Meters.of(0), Millimeters.of(207.372 + 50.8)),
            new Rotation3d(Degrees.of(0), newDeg, Degrees.of(0)));

        leftMotor.getSimState().setRawRotorPosition(Radians.of(sim.getAngleRads() * 100));
        leftMotor.getSimState().setRotorVelocity(RadiansPerSecond.of(sim.getVelocityRadPerSec() * 100));

        encoderSim.setPosition(-Units.degreesToRotations(newDeg.in(Degrees) - 29));
    }
}
