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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.simulation.PerfectAngleSim;

public class AngleSys extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(41, "canivore");
    private TalonFX rightMotor = new TalonFX(42, "canivore");

    private SparkMax sparkMaxEncoderOnly = new SparkMax(43, MotorType.kBrushed);
    private DigitalInput downLimit = new DigitalInput(9);
    private DigitalInput upLimit = new DigitalInput(8);

    private DIOSim downLimitSim;
    private DIOSim upLimitSim;

    private SparkRelativeEncoder encoder;
    private SparkRelativeEncoderSim encoderSim;

    public static PerfectAngleSim sim;

    @AutoLogOutput
    private Pose3d simPose = new Pose3d();
    @AutoLogOutput
    private Pose3d simFourBarOnePose = new Pose3d();
    @AutoLogOutput
    private Pose3d simFourBarTwoPose = new Pose3d();
    @AutoLogOutput
    private Pose3d zeroPose = new Pose3d();

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

    public static Angle fourBarConversion(Angle motorBarAngle) {
        // double sinTheta = Math.sin(motorBarAngle.in(Radians));
        // double cosTheta = Math.cos(motorBarAngle.in(Radians));

        // // Define the equation to solve for phi
        // UnivariateFunction equation = (double phi) -> {
        //     double cosPhi = Math.cos(phi);
        //     double sinPhi = Math.sin(phi);
        //     return Math.pow(450 * cosPhi - 380 - 150 * cosTheta, 2) + Math.pow(450 * sinPhi - 24.219 - 150 * sinTheta, 2) - Math.pow(200, 2);
        // };

        // // Use BrentSolver to find the root
        // BrentSolver solver = new BrentSolver(1e-10, 1e-8);
        // double lowerBound = Units.degreesToRadians(20);
        // double upperBound = Math.PI/2;

        // try {
        //     return Radians.of(solver.solve(1000, equation, lowerBound, upperBound));
        // } catch (Exception e) {
        //     System.out.println("Crashed THETA: " + motorBarAngle.in(Degrees));
        //     throw new IllegalArgumentException("No solution found for the given y.", e);
        // }

        // ----- The above code uses https://mvnrepository.com/artifact/org.apache.commons/commons-math3 and shows the original equation used to calculate PHI based on THETA.
        // By using the constraints and the solve function of TI-nspire CAS, we obtain the mathematical expression below.

        double y = motorBarAngle.in(Radians);
        return Radians.of(Math.atan(
            (Math.sin(y)+0.16146) /
            (Math.cos(y)+2.53333))
            
            - Math.asin(
                (0.375154*(Math.cos(y)+0.063734*(Math.sin(y)+45.417))) /
                (Math.sqrt(Math.cos(y)+0.063734*(Math.sin(y)+23.0517))))
            
            + Math.PI*0.5);
    }

    public void configureSimulation() {
        sim = new PerfectAngleSim(100, Degrees.of(-18), Degrees.of(107.78), Degrees.of(-18));
        
        leftMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        encoderSim = new SparkRelativeEncoderSim(sparkMaxEncoderOnly);

        upLimitSim = new DIOSim(upLimit);
        downLimitSim = new DIOSim(downLimit);
    }

    @Override
    public void simulationPeriodic() {
        leftMotor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        sim.setInputVoltage(leftMotor.getSimState().getMotorVoltage());
        sim.update(0.02);

        Logger.recordOutput("AngleSys/SimDeg", fourBarConversion(Radians.of(sim.getAngleRads())).in(Degrees));

        Angle newDeg = fourBarConversion(Radians.of(sim.getAngleRads())).plus(Degrees.of(10.3));

        // Translation values are obtained from the Robot CAD.
        simPose = new Pose3d(
            new Translation3d(Millimeters.of(308.198), Meters.of(0), Millimeters.of(207.372 + 50.8)),
            new Rotation3d(Degrees.of(0), newDeg, Degrees.of(0)));

        simFourBarOnePose = new Pose3d(
            new Translation3d(Millimeters.of(-73.6), Millimeters.of(0), Millimeters.of(231.6 + 50.8)),
            new Rotation3d(Degrees.of(0), Radians.of(-sim.getAngleRads()), Degrees.of(180))
        );
        // The simFourBarTwoPose is obtained by trignometry.
        simFourBarTwoPose = new Pose3d(
            new Translation3d(Millimeters.of(-73.6 - 150 * Math.cos(sim.getAngleRads())), Millimeters.of(0), Millimeters.of(231.6 + 50.8 + 150 * Math.sin(sim.getAngleRads()))),
            new Rotation3d(Degrees.of(0), Radians.of(-Math.asin((450 * Math.sin(fourBarConversion(Radians.of(sim.getAngleRads())).in(Radians)) - 24.219 - 150 * Math.sin(sim.getAngleRads())) / 200)), Degrees.of(0))
        );

        leftMotor.getSimState().setRawRotorPosition(Radians.of(sim.getAngleRads() * 100));
        leftMotor.getSimState().setRotorVelocity(RadiansPerSecond.of(sim.getVelocityRadPerSec() * 100));

        encoderSim.setPosition(-Units.degreesToRotations(newDeg.in(Degrees) - 29));

        double phi = AngleSys.fourBarConversion(Radians.of(AngleSys.sim.getAngleRads())).plus(Degrees.of(10.3)).in(Radians);
        Pose3d exitPose = new Pose3d(
            new Translation3d(Millimeters.of(+308.198 - 500 * Math.cos(phi)), Millimeters.of(0), Millimeters.of(207.372 + 50.8 + 500 * Math.sin(phi))),
            new Rotation3d(0,phi-Math.PI,0)
        );
        Logger.recordOutput("Debug/ExitPose", exitPose);

        upLimitSim.setValue(!sim.hasHitUpperLimit());
        downLimitSim.setValue(!sim.hasHitLowerLimit());
    }
}
