package frc.robot.commands.subsystems.angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AngleConstants;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.types.UpDownDirection;

public class RotateShooter extends Command {
    private final AngleSubystem angle;

    private final UpDownDirection direction;
    private final double speedPercentage;

    public RotateShooter(AngleSubystem angle, UpDownDirection direction, double speedPercentage) {
        this.angle = angle;
        this.direction = direction;
        this.speedPercentage = speedPercentage;

        addRequirements(angle);
    }

    @Override
    public void execute() {
        switch (direction) {
            default:
            case up:
                if (angle.getAngle() < AngleConstants.ANGLE_MAX)
                    angle.move(speedPercentage);
                else
                    angle.stop();
                break;
            case down:
                if (angle.getAngle() > AngleConstants.ANGLE_MIN)
                    angle.move(-speedPercentage);
                else
                    angle.stop();
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
