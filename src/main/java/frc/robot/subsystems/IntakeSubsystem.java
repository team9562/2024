package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax intake = new CANSparkMax(IntakeConstants.INTAKE_CAN, MotorType.kBrushless);

    public IntakeSubsystem() {
        intake.restoreFactoryDefaults();

        intake.setInverted(true);

        intake.enableVoltageCompensation(MotorConstants.NEO_V1_NOMINAL_VOLTAGE);

        intake.setIdleMode(IdleMode.kBrake);

        intake.setSmartCurrentLimit(IntakeConstants.STALL_LIMIT, MotorConstants.NEO_V1_FREE_LIMIT);
    }

    public void clearStickyFaults() {
        intake.clearFaults();
    }

    public void set(double speed) {
        intake.set(speed);
    }

    public void stop() {
        set(0);
    }
}
