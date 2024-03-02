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

        intake.enableVoltageCompensation(MotorConstants.NEO_550_NOMINAL_VOLTAGE);

        intake.setIdleMode(IdleMode.kBrake);

        intake.setSmartCurrentLimit(MotorConstants.NEO_550_STALL_LIMIT, MotorConstants.NEO_550_FREE_LIMIT);
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
