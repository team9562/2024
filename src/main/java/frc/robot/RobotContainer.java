// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));
  // private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  // private final IntakeSubsystem intake = new IntakeSubsystem();
  // private final ShooterSubsystem shooter = new ShooterSubsystem();
  // private final AngleSubystem angle = new AngleSubystem();

  private final PowerDistribution pdh = new PowerDistribution();

  Joystick driverYoke = new Joystick(1);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    Command yokeTeleopFieldRelative = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverYoke.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverYoke.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverYoke.getZ(), OperatorConstants.LEFT_X_DEADBAND), true);

    Command yokeTeleopRobotRelative = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverYoke.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverYoke.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverYoke.getZ(), OperatorConstants.LEFT_X_DEADBAND), false);

    m_chooser.addOption("Yoke TeleOp [Robot]", yokeTeleopRobotRelative);
    m_chooser.setDefaultOption("Yoke TeleOp [Field]", yokeTeleopFieldRelative);

    SmartDashboard.putData(m_chooser);

    drivebase.setDefaultCommand(m_chooser.getSelected());
  }

  public void clearStickyFaults() {
    pdh.clearStickyFaults();
    // elevator.clearStickyFaults();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(driverYoke, 11).onTrue((new InstantCommand(drivebase::zeroGyro)));
    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));

    // new JoystickButton(driverXbox,
        // 2).whileTrue(
            // Commands.deferredProxy(() -> drivebase.driveToPose(
                // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
    // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public void setDriveMode() {
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
