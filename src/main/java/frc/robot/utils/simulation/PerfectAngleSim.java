package frc.robot.utils.simulation;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class PerfectAngleSim {
    // Velocity and angle are relative to the external output, NOT the motor.
    private AngularVelocity velocity = RotationsPerSecond.of(0);
    private Angle angle;

    private final double gearing;
    private final Angle minAngle;
    private final Angle maxAngle;

    public PerfectAngleSim(double gearing, Angle minAngle, Angle maxAngle, Angle startingAngle) {
        this.gearing = gearing;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        if (startingAngle.gt(maxAngle)) this.angle = maxAngle;
        else if (startingAngle.lt(minAngle)) this.angle = minAngle;
        else this.angle = startingAngle;
    }

    public void setInputVoltage(double voltage) {
        // 6 Motor RPS per volt. THIS RELATIONSHIP IS NOT PHYSICALLY ACCURATE
        this.velocity = RotationsPerSecond.of(voltage * 6 / gearing);
    }

    public double getAngleRads() {
        return angle.in(Radians);
    }

    public double getVelocityRadPerSec() {
        return velocity.in(RadiansPerSecond);
    }

    public boolean hasHitLowerLimit() {
        return angle.lte(minAngle);
    }

    public boolean hasHitUpperLimit() {
        return angle.gte(maxAngle);
    }

    public void update(double dtSeconds) {
        Angle calculatedAngle = angle.plus(velocity.times(Seconds.of(dtSeconds)));
        if (calculatedAngle.gt(maxAngle)) {
            this.angle = maxAngle;
            this.velocity = RotationsPerSecond.of(0);
        }
        else if (calculatedAngle.lt(minAngle)) {
            this.angle = minAngle;
            this.velocity = RotationsPerSecond.of(0);
        }
        else angle = calculatedAngle;
    }
}
