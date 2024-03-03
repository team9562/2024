package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax elevatorLeft = new CANSparkMax(ElevatorConstants.LEFT_CAN, MotorType.kBrushless);
    private CANSparkMax elevatorRight = new CANSparkMax(ElevatorConstants.RIGHT_CAN, MotorType.kBrushless);

    private final PIDController elevatorPidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI,
            ElevatorConstants.kD);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS,
            ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    private RelativeEncoder elevatorLeftEncoder = elevatorLeft.getEncoder();
    private RelativeEncoder elevatorRightEncoder = elevatorRight.getEncoder();

    private DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_PORT);

    public ElevatorSubsystem() {
        elevatorLeft.restoreFactoryDefaults();
        elevatorRight.restoreFactoryDefaults();

        elevatorLeft.setInverted(true);

        elevatorRight.follow(elevatorLeft, true);

        elevatorLeft.setIdleMode(IdleMode.kBrake);
        elevatorRight.setIdleMode(IdleMode.kBrake);

        elevatorLeft.enableVoltageCompensation(MotorConstants.NEO_V1_NOMINAL_VOLTAGE);
        elevatorRight.enableVoltageCompensation(MotorConstants.NEO_V1_NOMINAL_VOLTAGE);

        elevatorLeft.setSmartCurrentLimit(MotorConstants.NEO_V1_STALL_LIMIT_HIGH, MotorConstants.NEO_V1_FREE_LIMIT);
        elevatorRight.setSmartCurrentLimit(MotorConstants.NEO_V1_STALL_LIMIT_HIGH, MotorConstants.NEO_V1_FREE_LIMIT);
    }

    public void clearStickyFaults() {
        elevatorLeft.clearFaults();
        elevatorRight.clearFaults();
    }

    public boolean isBottomedOut() {
        return limitSwitch.get();
    }

    // public void home() {
    //     while (!isBottomedOut()) {
    //         moveElevator(-0.1);
    //     }

    //     stop();
    //     resetElevatorEncoder();
    // }

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

    public void setElevatorPosition(double currentPosition, double setpoint) {
        double output = elevatorPidController.calculate(currentPosition, setpoint)
                + elevatorFeedforward.calculate(setpoint);

        if (output < -0.3)
            output = -0.3;
        else if (output > 0.3)
            output = 0.3;

        moveElevator(output);
    }

    // public void testElevatorPosition() {
    //     setElevatorPosition(elevatorLeftEncoder.getPosition(), 0.75 * 62);
    // }

    public void setElevatorPositionSpeed(double currentPosition, double setpoint, double limit) {
        double output = elevatorPidController.calculate(currentPosition, setpoint)
                + elevatorFeedforward.calculate(setpoint);

        if (output < -limit)
            output = -limit;
        else if (output > limit)
            output = limit;

        moveElevator(output);
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
        SmartDashboard.putNumber("Elevator Max Height", ElevatorConstants.MAX_HEIGHT);
        SmartDashboard.putNumber("Elevator Min Height", ElevatorConstants.MIN_HEIGHT);

        SmartDashboard.putBoolean("Bottomed Out", isBottomedOut());
    }
}
