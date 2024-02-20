package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax elevatorLeft = new CANSparkMax(ElevatorConstants.LEFT_CAN, MotorType.kBrushless);
    private CANSparkMax elevatorRight = new CANSparkMax(ElevatorConstants.RIGHT_CAN, MotorType.kBrushless);

    private final PIDController elevatorPidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

    private RelativeEncoder elevatorLeftEncoder = elevatorLeft.getEncoder();
    private RelativeEncoder elevatorRightEncoder = elevatorRight.getEncoder();

    public ElevatorSubsystem() {
        elevatorLeft.restoreFactoryDefaults();
        elevatorRight.restoreFactoryDefaults();

        elevatorLeft.setInverted(true);

        elevatorRight.follow(elevatorLeft);

        elevatorLeft.enableVoltageCompensation(8);
        elevatorRight.enableVoltageCompensation(8);
    }

    public void clearStickyFaults() {
        elevatorLeft.clearFaults();
        elevatorRight.clearFaults();
    }

    public double getElevatorHeight() {
        double sprocketPos = elevatorLeftEncoder.getPosition() / ElevatorConstants.kElevatorGearRatio;
        double circum = 2 * Math.PI * ElevatorConstants.kSprocketRadius;

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
        double output = elevatorPidController.calculate(currentPosition, setpoint) + elevatorFeedforward.calculate(setpoint);
    
        if (output < -0.3) output = -0.3;
        else if (output > 0.3) output = 0.3;

        moveElevator(output);
    }

    public void setElevatorPositionSpeed(double currentPosition, double setpoint, double limit) {
        double output = elevatorPidController.calculate(currentPosition, setpoint) + elevatorFeedforward.calculate(setpoint);

        if (output < -limit) output = -limit;
        else if (output > limit) output = limit;

        moveElevator(output);
    }

    public void stop() {
        elevatorLeft.stopMotor();
        elevatorRight.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Encoder Left", elevatorLeftEncoder.getPosition());
        SmartDashboard.putNumber("Elevator Encoder Right ", elevatorRightEncoder.getPosition());

        SmartDashboard.putNumber("Elevator Voltage Left", elevatorLeft.getBusVoltage());
        SmartDashboard.putNumber("Elevator Voltage Right", elevatorRight.getBusVoltage());
        
        SmartDashboard.putNumber("Elevator Output Left", elevatorLeft.getAppliedOutput());
        SmartDashboard.putNumber("Elevator Output Right", elevatorRight.getAppliedOutput());
    }
}
