package frc.robot.commands.subsystems.angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AngleConstants;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.types.AngleSetpoint;

public class RotateSetpoint extends Command {
    private final AngleSubystem angle;

    private final AngleSetpoint setpoint;

    public RotateSetpoint(AngleSubystem angle, AngleSetpoint setpoint) {
        this.angle = angle;
        this.setpoint = setpoint;

        addRequirements(angle);
    }

    @Override
    public void execute() {
        switch (setpoint) {
            default:
            case max:
                angle.setTargetAngle(AngleConstants.ANGLE_MAX);
                break; 
            case min:
                angle.setTargetAngle(AngleConstants.ANGLE_MIN);
        }
    }

    @Override
    public boolean isFinished() {
        return angle.isAtTargetAngle();
    }

    @Override
    public void end(boolean interrupted) {
        angle.stop();
    }
}
