package frc.robot.commands.angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.types.AngleSetpoint;
import frc.robot.types.IntakeState;
import frc.robot.util.Utility;

public class RotateSetpoint extends Command {
    private final AngleSubystem angle;
    private final ElevatorSubsystem elevator;
    private final IntakeSubsystem intake;

    private final AngleSetpoint setpoint;

    public RotateSetpoint(AngleSubystem angle, ElevatorSubsystem elevator, IntakeSubsystem intake, AngleSetpoint setpoint) {
        this.angle = angle;
        this.elevator = elevator;
        this.intake = intake;
        this.setpoint = setpoint;

        addRequirements(angle);
    }

    @Override
    public void execute() {
        boolean isSafe = !Utility.betweenRange(elevator.getEncoder() / ElevatorConstants.MAX_HEIGHT, ElevatorConstants.SAFE_MIN_PERCENT, ElevatorConstants.SAFE_MAX_PERCENT);

        if (isSafe && intake.getState() != IntakeState.intake)
            angle.setTargetAngle(setpoint.percentage);
    }

    @Override
    public boolean isFinished() {
        return false; // Angle sags when command is ended, so it runs infinitely for the time being
    }

    @Override
    public void end(boolean interrupted) {
        angle.stop();
    }
}
