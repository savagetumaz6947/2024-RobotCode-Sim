package frc.robot.commands.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class AutoAimNote extends PIDCommand {
    public AutoAimNote(CommandSwerveDrivetrain swerve, Vision intakeVision) {
        super(new PIDController(0.08, 0, 0), // TODO: testing
            () -> {
                if (intakeVision.hasTargets()) {
                    return intakeVision.getBestTarget().getYaw();
                } else {
                    return 0;
                }
            },
            0,
            (double vyMetersPerSecond) -> {
                swerve.applyRequest(new ChassisSpeeds(0, vyMetersPerSecond, 0));
            },
            swerve
        );
        getController().setTolerance(1);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
