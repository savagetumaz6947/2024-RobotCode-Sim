package frc.robot.commands.Angle;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.AngleSys;

@SuppressWarnings("removal")
public class RiseToAngle extends PIDCommand {
    public RiseToAngle(DoubleSupplier targetAngleDegrees, AngleSys angleSys) {
        super(
            new PIDController(.5, 0, 0.001),
            angleSys::getAngle,
            targetAngleDegrees,
            angleSys::move,
            angleSys
        );
        getController().setTolerance(0.3);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

    @Override
    public void execute() {
        super.execute();
        // Logger.recordOutput("Commands/Angle/RiseToAngle/Setpoint", getController().getSetpoint());
        // Logger.recordOutput("Commands/Angle/RiseToAngle/AtSetpoint", getController().atSetpoint());
    }

}
