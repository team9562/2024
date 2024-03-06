package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.Utility;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax elevatorLeft = new CANSparkMax(ElevatorConstants.LEFT_CAN, MotorType.kBrushless);
    private CANSparkMax elevatorRight = new CANSparkMax(ElevatorConstants.RIGHT_CAN, MotorType.kBrushless);

    private final SparkPIDController elevatorPidController = elevatorLeft.getPIDController();

    private RelativeEncoder elevatorLeftEncoder = elevatorLeft.getEncoder();
    private RelativeEncoder elevatorRightEncoder = elevatorRight.getEncoder();

    private DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_PORT);

    private double targetHeight;

    public ElevatorSubsystem() {
        elevatorLeft.restoreFactoryDefaults();
        elevatorRight.restoreFactoryDefaults();

        elevatorLeft.setInverted(true);

        elevatorRight.follow(elevatorLeft, true);

        elevatorLeft.setIdleMode(IdleMode.kBrake);
        elevatorRight.setIdleMode(IdleMode.kBrake);

        elevatorLeft.enableVoltageCompensation(MotorConstants.NEO_V1_NOMINAL_VOLTAGE);
        elevatorRight.enableVoltageCompensation(MotorConstants.NEO_V1_NOMINAL_VOLTAGE);

        elevatorLeft.setSmartCurrentLimit(ElevatorConstants.STALL_LIMIT, MotorConstants.NEO_V1_FREE_LIMIT);
        elevatorRight.setSmartCurrentLimit(ElevatorConstants.STALL_LIMIT, MotorConstants.NEO_V1_FREE_LIMIT);

        elevatorPidController.setFeedbackDevice(elevatorLeftEncoder);

        elevatorPidController.setP(ElevatorConstants.kP);
        elevatorPidController.setI(ElevatorConstants.kI);
        elevatorPidController.setD(ElevatorConstants.kD);
        elevatorPidController.setFF(ElevatorConstants.kFF);
    }

    public void clearStickyFaults() {
        elevatorLeft.clearFaults();
        elevatorRight.clearFaults();
    }

    public boolean isBottomedOut() {
        return limitSwitch.get();
    }

    public double getElevatorHeight() {
        double sprocketPos = elevatorLeftEncoder.getPosition() / ElevatorConstants.GEAR_RATIO;
        double circum = 2 * Math.PI * ElevatorConstants.SPROCKET_RADIUS;

        return sprocketPos * circum;
    }

    public void moveElevator(double output) {
        elevatorLeft.set(output);
    }

    public void resetElevatorEncoder() {
        elevatorLeftEncoder.setPosition(0);
        elevatorRightEncoder.setPosition(0);
    }

    public void setElevatorPosition(double setpointPercentage) {
        targetHeight = setpointPercentage * ElevatorConstants.MAX_HEIGHT;

        elevatorPidController.setReference(targetHeight, ControlType.kPosition);
    }

    public boolean isAtTargetHeight() {
        return Utility.withinTolerance(elevatorLeftEncoder.getPosition(), targetHeight, ElevatorConstants.ELEVATOR_THRESHOLD);
    }

    public void stop() {
        elevatorLeft.stopMotor();
        elevatorRight.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder Left", elevatorLeftEncoder.getPosition());
        SmartDashboard.putNumber("Elevator Encoder Right ", elevatorRightEncoder.getPosition());

        SmartDashboard.putNumber("Elevator Voltage Left", elevatorLeft.getBusVoltage());
        SmartDashboard.putNumber("Elevator Voltage Right", elevatorRight.getBusVoltage());

        SmartDashboard.putNumber("Elevator Output Left", elevatorLeft.getAppliedOutput());
        SmartDashboard.putNumber("Elevator Output Right", elevatorRight.getAppliedOutput());

        SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
        SmartDashboard.putNumber("Elevator Target Height", targetHeight);
        SmartDashboard.putBoolean("Elevator Is At Target Height", isAtTargetHeight());
        SmartDashboard.putNumber("Elevator Max Height", ElevatorConstants.MAX_HEIGHT);
        SmartDashboard.putNumber("Elevator Min Height", ElevatorConstants.MIN_HEIGHT);

        SmartDashboard.putBoolean("Bottomed Out", isBottomedOut());
    }
}
