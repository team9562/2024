package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
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

    public AngleSubystem() {
        angle.restoreFactoryDefaults();

        angle.setInverted(true);

        angle.setIdleMode(IdleMode.kBrake);

        angle.enableVoltageCompensation(MotorConstants.NEO_V1_NOMINAL_VOLTAGE);

        angle.setSmartCurrentLimit(MotorConstants.NEO_V1_STALL_LIMIT_LOW, MotorConstants.NEO_V1_FREE_LIMIT);

        // anglePidController.setFeedbackDevice(angle.getEncoder());

        anglePidController.setP(AngleConstants.kP);
        anglePidController.setI(AngleConstants.kI);
        anglePidController.setD(AngleConstants.kD);
        anglePidController.setFF(AngleConstants.kFF);
    }
    
    public void clearStickyFaults() {
        angle.clearFaults();
    }
    
    public void setTargetAngle(double angle) {
        if (angle > AngleConstants.ANGLE_MAX) targetAngle = AngleConstants.ANGLE_MAX;
        else if (angle < AngleConstants.ANGLE_MIN) targetAngle = AngleConstants.ANGLE_MIN;
        else targetAngle = angle;

        // anglePidController.setReference(targetAngle, ControlType.kPosition);
    }
    
    public void move(double speed) {
        angle.set(speed);
        // anglePidController.setReference(speed, ControlType.kVelocity);
    }
    
    public double getAngle() {
        return angleEncoder.getAbsolutePosition();
    }
    
    public boolean isAtTargetAngle() {
        return Utility.withinTolerance(getAngle(), targetAngle, AngleConstants.ANGLE_THRESHOLD);
    }
    
    public void stop() {
        angle.set(0);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Target Angle", targetAngle);

        SmartDashboard.putNumber("Angle Absolute Encoder", getAngle());
        SmartDashboard.putNumber("Angle Integrated Encoder", angle.getEncoder().getPosition());

        SmartDashboard.putBoolean("Encoder Connected", angleEncoder.isConnected());
    
        SmartDashboard.putBoolean("At Target Angle", isAtTargetAngle());
    }
}
