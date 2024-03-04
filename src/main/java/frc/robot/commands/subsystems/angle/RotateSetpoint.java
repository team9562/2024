package frc.robot.commands.subsystems.angle;

import edu.wpi.first.wpilibj2.command.Command;
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
                angle.setTargetAngle(1);
                break;
            case min:    
                angle.setTargetAngle(0);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        angle.stop();
    }
}
