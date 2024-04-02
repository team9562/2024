package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivebaseConstants.TurnAroundPIDConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class FaceAngleCommand extends Command {
    private final SwerveSubsystem swerve;
    private final PIDController pid = new PIDController(TurnAroundPIDConstants.kP, TurnAroundPIDConstants.kI, TurnAroundPIDConstants.kD);
    private final double angle;

    public FaceAngleCommand(SwerveSubsystem swerve, double angle) {
        this.swerve = swerve;
        this.angle = angle;

        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
        pid.setSetpoint(angle);
        pid.setTolerance(TurnAroundPIDConstants.TOLERANCE);
        pid.enableContinuousInput(0, 360);
    }

    @Override
    public void execute() {
        System.out.println(angle);
        double output = pid.calculate(swerve.getHeading().getDegrees());
        swerve.drive(new Translation2d(0, 0), output, false);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0));
    }
}
