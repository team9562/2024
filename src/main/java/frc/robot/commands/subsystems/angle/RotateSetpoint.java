package frc.robot.commands.subsystems.angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.types.AngleSetpoint;
import frc.robot.util.Utility;

public class RotateSetpoint extends Command {
    private final AngleSubystem angle;
    private final ElevatorSubsystem elevator;

    private final AngleSetpoint setpoint;

    public RotateSetpoint(AngleSubystem angle, ElevatorSubsystem elevator, AngleSetpoint setpoint) {
        this.angle = angle;
        this.elevator = elevator;
        this.setpoint = setpoint;

        addRequirements(angle);
    }

    @Override
    public void execute() {
        boolean isSafe = !Utility.betweenRange(elevator.getEncoder() / ElevatorConstants.MAX_HEIGHT, ElevatorConstants.SAFE_MIN_PERCENT, ElevatorConstants.SAFE_MAX_PERCENT);

        if (isSafe)
            angle.setTargetAngle(setpoint.percentage);
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
