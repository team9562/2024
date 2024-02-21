package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.Utility;

public class AngleSubystem extends SubsystemBase {
    private CANSparkMax angle = new CANSparkMax(AngleConstants.ANGLE_CAN, MotorType.kBrushless);

    private SparkPIDController anglePidController = angle.getPIDController();
    
    private double targetAngleDegrees;

    public AngleSubystem() {
        angle.restoreFactoryDefaults();

        angle.setIdleMode(IdleMode.kBrake);

        angle.enableVoltageCompensation(MotorConstants.NEO_V1_NOMINAL_VOLTAGE);

        angle.setSmartCurrentLimit(MotorConstants.NEO_V1_STALL_LIMIT_LOW, MotorConstants.NEO_V1_FREE_LIMIT);

        anglePidController.setP(AngleConstants.kP);
        anglePidController.setI(AngleConstants.kI);
        anglePidController.setD(AngleConstants.kD);
        anglePidController.setFF(AngleConstants.kFF);
    }

    public void clearStickyFaults() {
        angle.clearFaults();
    }

    public void setTargetAngleDegrees(double angle) {
        if (angle > AngleConstants.ANGLE_MAX) targetAngleDegrees = AngleConstants.ANGLE_MAX;
        else if (angle < AngleConstants.ANGLE_MIN) targetAngleDegrees = AngleConstants.ANGLE_MIN;
        else targetAngleDegrees = angle;

        anglePidController.setReference(targetAngleDegrees / 360, ControlType.kPosition);
    }

    public double getAngleDegrees() {
        return angle.getEncoder().getPosition() * 360;
    }

    public boolean isAtTargetAngle() {
        return Utility.withinTolerance(getAngleDegrees(), targetAngleDegrees, AngleConstants.ANGLE_THRESHOLD_DEGREES);
    }

    public void stop() {
        angle.set(0);
    }
}
