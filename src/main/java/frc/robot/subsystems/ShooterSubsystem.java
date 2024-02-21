package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Utility;

public class ShooterSubsystem extends SubsystemBase {
        private Rev2mDistanceSensor distance = new Rev2mDistanceSensor(Port.kOnboard);

        private CANSparkMax shooterFeeder = new CANSparkMax(ShooterConstants.FEEDER_CAN, MotorType.kBrushless);
        private CANSparkMax shooterTop = new CANSparkMax(ShooterConstants.TOP_CAN, MotorType.kBrushless);
        private CANSparkMax shooterBottom = new CANSparkMax(ShooterConstants.BOTTOM_CAN, MotorType.kBrushless);
        
        private SparkPIDController feederPidController = shooterFeeder.getPIDController();
        private SparkPIDController topPidController = shooterTop.getPIDController();
        private SparkPIDController bottomPidController = shooterBottom.getPIDController();
        
        private double targetTop;
        private double targetBottom;
        
        public ShooterSubsystem() {
            distance.setDistanceUnits(Unit.kInches);

            shooterFeeder.restoreFactoryDefaults();
            shooterTop.restoreFactoryDefaults();
            shooterBottom.restoreFactoryDefaults();

            shooterTop.setInverted(true);

            shooterBottom.follow(shooterTop);

            shooterFeeder.setIdleMode(IdleMode.kBrake);
            shooterTop.setIdleMode(IdleMode.kCoast);
            shooterBottom.setIdleMode(IdleMode.kCoast);

            shooterFeeder.enableVoltageCompensation(MotorConstants.NEO_550_NOMINAL_VOLTAGE);
            shooterTop.enableVoltageCompensation(MotorConstants.NEO_V1_NOMINAL_VOLTAGE);
            shooterBottom.enableVoltageCompensation(MotorConstants.NEO_V1_NOMINAL_VOLTAGE);

            shooterFeeder.setSmartCurrentLimit(MotorConstants.NEO_550_STALL_LIMIT, MotorConstants.NEO_550_FREE_LIMIT);
            shooterTop.setSmartCurrentLimit(MotorConstants.NEO_V1_STALL_LIMIT_LOW, MotorConstants.NEO_V1_FREE_LIMIT);
            shooterBottom.setSmartCurrentLimit(MotorConstants.NEO_V1_STALL_LIMIT_LOW, MotorConstants.NEO_V1_FREE_LIMIT);

            feederPidController.setP(ShooterConstants.kP);
            feederPidController.setI(ShooterConstants.kI);
            feederPidController.setD(ShooterConstants.kD);
            feederPidController.setFF(ShooterConstants.kFF);

            topPidController.setP(ShooterConstants.kP);
            topPidController.setI(ShooterConstants.kI);
            topPidController.setD(ShooterConstants.kD);
            topPidController.setFF(ShooterConstants.kFF);

            bottomPidController.setP(ShooterConstants.kP);
            bottomPidController.setI(ShooterConstants.kI);
            bottomPidController.setD(ShooterConstants.kD);
            bottomPidController.setFF(ShooterConstants.kFF);

            feederPidController.setOutputRange(-1, 1);
            topPidController.setOutputRange(-1, 1);
            bottomPidController.setOutputRange(-1, 1);
        }

        public void clearStickyFaults() {
            shooterFeeder.clearFaults();
            shooterTop.clearFaults();
            shooterBottom.clearFaults();
        }

        public void setRPMs(double top, double bottom) {
            targetTop = top;
            targetBottom = bottom;

            topPidController.setReference(top, ControlType.kVelocity);
            bottomPidController.setReference(bottom, ControlType.kVelocity);
        }

        public void setFeeder(double feeder) {
            feederPidController.setReference(feeder, ControlType.kVelocity);
        }

        public double getTopVelocity() {
            return shooterTop.getEncoder().getVelocity();
        }

        public double getBottomVelocity() {
            return shooterBottom.getEncoder().getVelocity();
        }

        public boolean isAtTargetSpeed() {
            return Utility.withinTolerance(getTopVelocity(), targetTop, 100)
                    && Utility.withinTolerance(getBottomVelocity(), targetBottom, 100);
        }

        public void stop() {
            shooterFeeder.set(0);
            shooterTop.set(0);
            shooterBottom.set(0);
        }

        @Override
        public void periodic() {
            SmartDashboard.putBoolean("Note Loaded", isNoteLoaded());
            SmartDashboard.putBoolean("At Target Speed", isAtTargetSpeed());
            
            SmartDashboard.putNumber("Target Speed Top", targetTop);
            SmartDashboard.putNumber("Target Speed Bottom", targetBottom);
        }
        
        public boolean isNoteLoaded() {
            return distance.getRange() <= ShooterConstants.SENSOR_THRESHOLD_INCHES;
        }
    }
