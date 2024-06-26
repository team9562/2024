package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.Utility;

public class AngleSubystem extends SubsystemBase {
    private CANSparkMax angle = new CANSparkMax(AngleConstants.ANGLE_CAN, MotorType.kBrushless);

    private DutyCycleEncoder angleEncoder = new DutyCycleEncoder(AngleConstants.ENCODER_PORT);

    private SparkPIDController anglePidController = angle.getPIDController();

    private double targetAngle;
    private boolean isMin;

    public AngleSubystem() {
        angle.restoreFactoryDefaults();

        angle.setInverted(true);

        // angle.setIdleMode(IdleMode.kCoast);
        angle.setIdleMode(IdleMode.kBrake);

        angle.enableVoltageCompensation(MotorConstants.NEO_V1_NOMINAL_VOLTAGE);

        angle.setSmartCurrentLimit(AngleConstants.STALL_LIMIT, MotorConstants.NEO_V1_FREE_LIMIT);

        anglePidController.setFeedbackDevice(angle.getEncoder());

        anglePidController.setP(AngleConstants.kP);
        anglePidController.setI(AngleConstants.kI);
        anglePidController.setD(AngleConstants.kD);
        anglePidController.setFF(AngleConstants.kFF);
    }

    public void homeAngle() {
        double absPercent = (getAngle() - AngleConstants.ANGLE_MIN_ABS) / (AngleConstants.ANGLE_MAX_ABS - AngleConstants.ANGLE_MIN_ABS);
        angle.getEncoder().setPosition(absPercent * AngleConstants.ANGLE_MAX_REL);
    }

    public void clearStickyFaults() {
        angle.clearFaults();
    }

    public void burnFlash() {
        angle.burnFlash();
    }

    public boolean isAtMin() {
        return isMin && isAtTargetAngle();
    }

    public void setTargetAnglePercentage(double anglePercentage) {
        isMin = anglePercentage == 0;

        targetAngle = anglePercentage * AngleConstants.ANGLE_MAX_REL;
        anglePidController.setReference(targetAngle, ControlType.kPosition);
    }

    public void setTargetAngleDegrees(double degrees) {
        isMin = degrees == 0;

        if (degrees > AngleConstants.ANGLE_MAX_REL) degrees = AngleConstants.ANGLE_MAX_REL;
        else if (degrees < AngleConstants.ANGLE_MIN_REL) degrees = AngleConstants.ANGLE_MIN_REL;

        targetAngle = degrees;

        anglePidController.setReference(targetAngle, ControlType.kPosition);
    }

    public void move(double speed) {
        angle.set(speed);
    }

    public double getAngle() {
        return angleEncoder.get();
    }

    public boolean isAtTargetAngle() {
        return Utility.withinTolerance(angle.getEncoder().getPosition(), targetAngle, AngleConstants.ANGLE_THRESHOLD);
    }

    public void resetRelativeEncoder() {
        angle.getEncoder().setPosition(0);
    }

    public void stop() {
        angle.set(0);
    }

    public void lock(boolean lock) {
        if (lock)
            angle.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Target Angle", targetAngle);
        SmartDashboard.putNumber("Angle Current", angle.getOutputCurrent());
        SmartDashboard.putNumber("Angle Absolute Encoder", getAngle());
        SmartDashboard.putNumber("Angle Relative Encoder", angle.getEncoder().getPosition());
        
        SmartDashboard.putBoolean("Angle At Min", isAtMin());
        SmartDashboard.putBoolean("Angle Absolute Encoder Connected", angleEncoder.isConnected());
        SmartDashboard.putBoolean("At Target Angle", isAtTargetAngle());
    }
}
