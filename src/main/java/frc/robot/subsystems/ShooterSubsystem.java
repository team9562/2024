package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Utility;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax shooterFeeder = new CANSparkMax(ShooterConstants.FEEDER_CAN, MotorType.kBrushless);
    private CANSparkMax shooterLeft = new CANSparkMax(ShooterConstants.LEFT_CAN, MotorType.kBrushless);
    private CANSparkMax shooterRight = new CANSparkMax(ShooterConstants.RIGHT_CAN, MotorType.kBrushless);

    private SparkPIDController feederPidController = shooterFeeder.getPIDController();
    private SparkPIDController leftPidController = shooterLeft.getPIDController();
    private SparkPIDController rightPidController = shooterRight.getPIDController();

    private DigitalInput limitSwitch = new DigitalInput(ShooterConstants.LIMIT_SWITCH_PORT);

    private double targetRPMs;
    private double targetFeeder;

    public ShooterSubsystem() {
        shooterFeeder.restoreFactoryDefaults();
        shooterLeft.restoreFactoryDefaults();
        shooterRight.restoreFactoryDefaults();

        shooterLeft.setInverted(false);
        shooterRight.setInverted(false);
        shooterFeeder.setInverted(false);

        shooterFeeder.setIdleMode(IdleMode.kCoast);
        shooterLeft.setIdleMode(IdleMode.kCoast);
        shooterRight.setIdleMode(IdleMode.kCoast);

        shooterFeeder.enableVoltageCompensation(MotorConstants.NEO_550_NOMINAL_VOLTAGE);
        shooterLeft.enableVoltageCompensation(MotorConstants.NEO_V1_NOMINAL_VOLTAGE);
        shooterRight.enableVoltageCompensation(MotorConstants.NEO_V1_NOMINAL_VOLTAGE);

        shooterFeeder.setSmartCurrentLimit(ShooterConstants.FEEDER_STALL_LIMIT, MotorConstants.NEO_550_FREE_LIMIT);
        shooterLeft.setSmartCurrentLimit(ShooterConstants.STALL_LIMIT, MotorConstants.NEO_V1_FREE_LIMIT);
        shooterRight.setSmartCurrentLimit(ShooterConstants.STALL_LIMIT, MotorConstants.NEO_V1_FREE_LIMIT);

        feederPidController.setP(ShooterConstants.kP_FEEDER);
        feederPidController.setI(ShooterConstants.kI_FEEDER);
        feederPidController.setD(ShooterConstants.kD_FEEDER);
        feederPidController.setFF(ShooterConstants.kFF_FEEDER);

        leftPidController.setP(ShooterConstants.kP_LEFT);
        leftPidController.setI(ShooterConstants.kI_LEFT);
        leftPidController.setD(ShooterConstants.kD_LEFT);
        leftPidController.setFF(ShooterConstants.kFF_LEFT);

        rightPidController.setP(ShooterConstants.kP_RIGHT);
        rightPidController.setI(ShooterConstants.kI_RIGHT);
        rightPidController.setD(ShooterConstants.kD_RIGHT);
        rightPidController.setFF(ShooterConstants.kFF_RIGHT);

        feederPidController.setOutputRange(-1, 1);
        leftPidController.setOutputRange(-1, 1);
        rightPidController.setOutputRange(-1, 1);
    }

    public void clearStickyFaults() {
        shooterFeeder.clearFaults();
        shooterLeft.clearFaults();
        shooterRight.clearFaults();
    }

    public void burnFlash() {
        shooterFeeder.burnFlash();
        shooterLeft.burnFlash();
        shooterRight.burnFlash();
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

    public boolean isBottomedOut() {
        return !limitSwitch.get(); // Normally open
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

        leftPidController.setReference(-targetRPMs, ControlType.kVelocity);
        rightPidController.setReference(targetRPMs, ControlType.kVelocity);
    }

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

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("At Target Velocity", isAtTargetVelocity());

        SmartDashboard.putBoolean("Shooter Bottomed Out", isBottomedOut());

        SmartDashboard.putNumber("Target Velocity", targetRPMs);
        SmartDashboard.putNumber("Target Velocity Feeder", targetFeeder);

        SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
        SmartDashboard.putNumber("Right Velocity", getRightVelocity());
        SmartDashboard.putNumber("Feeder Velocity", getFeederVelocity());
    }
}
