package frc.robot.commands.angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.subsystems.AprilTagsVisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.types.IntakeState;
import frc.robot.util.Utility;

public class AutoSpeakerAngleCommand extends Command {
    private final AngleSubystem angle;
    private final ElevatorSubsystem elevator;
    private final IntakeSubsystem intake;
    private final AprilTagsVisionSubsystem aprilTagsVision;
    private final boolean auto;

    public AutoSpeakerAngleCommand(AngleSubystem angle, ElevatorSubsystem elevator, IntakeSubsystem intake, AprilTagsVisionSubsystem aprilTagsVision, boolean auto) {
        this.angle = angle;
        this.elevator = elevator;
        this.intake = intake;
        this.aprilTagsVision = aprilTagsVision;
        this.auto = auto;

        addRequirements(angle);
    }

    @Override
    public void execute() {
        boolean isSafe = !Utility.betweenRange(elevator.getEncoder() / ElevatorConstants.MAX_HEIGHT, ElevatorConstants.SAFE_MIN_PERCENT, ElevatorConstants.SAFE_MAX_PERCENT);

        if (isSafe && intake.getState() != IntakeState.intake)
            angle.setTargetAngleDegrees(aprilTagsVision.calculateShooterAngle());
    }

    @Override
    public boolean isFinished() {
        return auto ? angle.isAtTargetAngle() : false;
    }

    @Override
    public void end(boolean interrupted) {
        angle.stop();
    }
}
