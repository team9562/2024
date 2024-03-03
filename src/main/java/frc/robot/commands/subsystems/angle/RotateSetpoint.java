package frc.robot.commands.subsystems.angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AngleConstants;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.types.AngleSetpoint;
import frc.robot.types.UpDownDirection;

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
        UpDownDirection direction = UpDownDirection.up;

        switch (setpoint) {
            default:
                break;
            case max:
                angle.setTargetAngle(AngleConstants.ANGLE_MAX);
                if (angle.getAngle() < AngleConstants.ANGLE_MAX)
                    direction = UpDownDirection.up;
                else
                    direction = UpDownDirection.down;
                break;
            case min:
                angle.setTargetAngle(AngleConstants.ANGLE_MIN);
                if (angle.getAngle() < AngleConstants.ANGLE_MIN)
                    direction = UpDownDirection.up;
                else
                    direction = UpDownDirection.down;
                break;
        }

        if (!angle.isAtTargetAngle())
            angle.move(direction == UpDownDirection.up ? 0.6 : -0.6);
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
