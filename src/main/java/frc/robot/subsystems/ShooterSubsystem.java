package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Utility;

public class ShooterSubsystem extends SubsystemBase {
    private Rev2mDistanceSensor distance;

    private CANSparkMax shooterFeeder = new CANSparkMax(ShooterConstants.FEEDER_CAN, MotorType.kBrushless);
    private CANSparkMax shooterLeft = new CANSparkMax(ShooterConstants.LEFT_CAN, MotorType.kBrushless);
    private CANSparkMax shooterRight = new CANSparkMax(ShooterConstants.RIGHT_CAN, MotorType.kBrushless);

    private SparkPIDController feederPidController = shooterFeeder.getPIDController();
    private SparkPIDController leftPidController = shooterLeft.getPIDController();
    private SparkPIDController rightPidController = shooterRight.getPIDController();

    private double targetRPMs;
    private double targetFeeder;

    public ShooterSubsystem(Rev2mDistanceSensor distanceSensor) {
        distance = distanceSensor;

        shooterFeeder.restoreFactoryDefaults();
        shooterLeft.restoreFactoryDefaults();
        shooterRight.restoreFactoryDefaults();

        shooterLeft.setInverted(false);
        shooterRight.setInverted(false);
        shooterFeeder.setInverted(false);

        // shooterRight.follow(shooterLeft, false);

        shooterFeeder.setIdleMode(IdleMode.kCoast);
        shooterLeft.setIdleMode(IdleMode.kCoast);
        shooterRight.setIdleMode(IdleMode.kCoast);

        shooterFeeder.enableVoltageCompensation(MotorConstants.NEO_550_NOMINAL_VOLTAGE);
        shooterLeft.enableVoltageCompensation(MotorConstants.NEO_V1_NOMINAL_VOLTAGE);
        shooterRight.enableVoltageCompensation(MotorConstants.NEO_V1_NOMINAL_VOLTAGE);

        shooterFeeder.setSmartCurrentLimit(ShooterConstants.FEEDER_STALL_LIMIT, MotorConstants.NEO_550_FREE_LIMIT);
        shooterLeft.setSmartCurrentLimit(ShooterConstants.STALL_LIMIT, MotorConstants.NEO_V1_FREE_LIMIT);
        shooterRight.setSmartCurrentLimit(ShooterConstants.STALL_LIMIT, MotorConstants.NEO_V1_FREE_LIMIT);

        feederPidController.setP(ShooterConstants.kP);
        feederPidController.setI(ShooterConstants.kI);
        feederPidController.setD(ShooterConstants.kD);
        feederPidController.setFF(ShooterConstants.kFF);

        leftPidController.setP(ShooterConstants.kP);
        leftPidController.setI(ShooterConstants.kI);
        leftPidController.setD(ShooterConstants.kD);
        leftPidController.setFF(ShooterConstants.kFF);

        rightPidController.setP(ShooterConstants.kP);
        rightPidController.setI(ShooterConstants.kI);
        rightPidController.setD(ShooterConstants.kD);
        rightPidController.setFF(ShooterConstants.kFF);

        feederPidController.setOutputRange(-1, 1);
        leftPidController.setOutputRange(-1, 1);
        rightPidController.setOutputRange(-1, 1);
    }

    public void clearStickyFaults() {
        shooterFeeder.clearFaults();
        shooterLeft.clearFaults();
        shooterRight.clearFaults();
    }

    public void setRPMs(double rpms) {
        targetRPMs = rpms;

        leftPidController.setReference(targetRPMs, ControlType.kVelocity);
        rightPidController.setReference(targetRPMs, ControlType.kVelocity);
    }

    public void setFeeder(double feeder) {
        targetFeeder = feeder;

        feederPidController.setReference(targetFeeder, ControlType.kVelocity);
    }

    public double getLeftVelocity() {
        return shooterLeft.getEncoder().getVelocity();
    }

    public double getRightVelocity() {
        return shooterRight.getEncoder().getVelocity();
    }

    public double getFeederVelocity() {
        return shooterFeeder.getEncoder().getVelocity();
    }

    public boolean isAtTargetVelocity() {
        return Utility.withinTolerance(getLeftVelocity(), targetRPMs, 100);
    }

    public void shootAmpMaxSpeed() {
        targetRPMs = MotorConstants.NEO_V1_MAX_RPMS;

        leftPidController.setReference(targetRPMs, ControlType.kVelocity);
        rightPidController.setReference(-targetRPMs, ControlType.kVelocity);
    }

    /**
     * 
     */
    public void stopShooters() {
        targetRPMs = 0;

        shooterLeft.stopMotor();
        shooterRight.stopMotor();
    }

    public void stopFeeder() {
        targetFeeder = 0;

        shooterFeeder.set(0);
    }

    public void stopAll() {
        stopShooters();
        stopFeeder();
    }

    public boolean isNoteLoaded() {
        return distance.getRange() <= ShooterConstants.SENSOR_THRESHOLD_INCHES;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Note Loaded", isNoteLoaded());
        SmartDashboard.putNumber("Note Distance", distance.getRange());
        SmartDashboard.putBoolean("Note Distance Valid", distance.isRangeValid());
        SmartDashboard.putNumber("Note Distance Timestamp", distance.getTimestamp());
        SmartDashboard.putBoolean("At Target Velocity", isAtTargetVelocity());

        SmartDashboard.putNumber("Target Velocity", targetRPMs);
        SmartDashboard.putNumber("Target Velocity Feeder", targetFeeder);

        SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
        SmartDashboard.putNumber("Right Velocity", getRightVelocity());
        SmartDashboard.putNumber("Feeder Velocity", getFeederVelocity());
    }
}
