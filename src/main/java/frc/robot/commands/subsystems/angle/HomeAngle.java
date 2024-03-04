package frc.robot.commands.subsystems.angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AngleConstants;
import frc.robot.subsystems.AngleSubystem;

public class HomeAngle extends Command {
    private final AngleSubystem angle;

    public HomeAngle(AngleSubystem angle) {
        this.angle = angle;
    }

    @Override
    public void initialize() {
        angle.stop();
    }

    @Override
    public void execute() {
        if (angle.getAngle() > AngleConstants.ANGLE_MIN_ABS)
            angle.move(-0.1);
    }

    @Override
    public boolean isFinished() {
        return angle.getAngle() <= AngleConstants.ANGLE_MIN_ABS;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted)
            angle.resetRelativeEncoder();

        angle.stop();
    }
}
