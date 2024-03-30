// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.angle.RotateSetpoint;
import frc.robot.commands.elevator.MoveSetpoint;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.shooter.Feed;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootAmp;
import frc.robot.commands.shooter.ShooterIntake;
import frc.robot.commands.swerve.AutoIntakeCommand;
import frc.robot.commands.swerve.TurnAroundCommand;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.types.AngleSetpoint;
import frc.robot.types.ElevatorSetpoint;
import frc.robot.types.InOutDirection;
import frc.robot.types.SpeakerPosition;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter;
  private final AngleSubystem angle = new AngleSubystem();

  private final PowerDistribution pdh = new PowerDistribution();

  CommandJoystick driverYoke = new CommandJoystick(1);
  CommandXboxController driverXbox = new CommandXboxController(0);

  private final SendableChooser<Command> m_teleopChooser = new SendableChooser<>();
  private final SendableChooser<Command> m_autoChooser;

  private final Command zeroGyroCommand = new InstantCommand(drivebase::zeroGyro);
  private final Command turnAroundCommand = new TurnAroundCommand(drivebase);
  private final Command autoIntakeCommand;
  // private final Command homeElevatorCommand = new HomeElevator(elevator,
  // angle);
  private final Command elevatorMinCommand = new MoveSetpoint(elevator, angle, ElevatorSetpoint.min);
  // private final Command elevatorHangCommand = new MoveSetpoint(elevator, angle,
  // ElevatorSetpoint.hang);
  private final Command elevatorHalfCommand = new MoveSetpoint(elevator, angle, ElevatorSetpoint.half);
  private final Command elevatorMaxCommand = new MoveSetpoint(elevator, angle, ElevatorSetpoint.max);
  private final Command angleMinCommand = new RotateSetpoint(angle, elevator, intake, AngleSetpoint.min);// .withTimeout(1);
  private final Command angleHalfCommand = new RotateSetpoint(angle, elevator, intake, AngleSetpoint.half);// .withTimeout(1);
  private final Command anglePodiumCommand = new RotateSetpoint(angle, elevator, intake, AngleSetpoint.podium);// .withTimeout(1);
  private final Command angleMaxCommand = new RotateSetpoint(angle, elevator, intake, AngleSetpoint.max);// .withTimeout(1);
  private final Command shooterShootCommand;
  private final Command shooterShootAmpCommand;
  private final Command shooterIntakeCommand;
  private final Command shooterFeedCommand;
  private final Command shootSequenceCommand;
  private final Command intakeInCommand = new Intake(intake, InOutDirection.in);
  private final Command intakeOutCommand = new Intake(intake, InOutDirection.out);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    shooter = new ShooterSubsystem();

    shooterShootCommand = new Shoot(shooter);
    shooterShootAmpCommand = new ShootAmp(shooter);
    shooterIntakeCommand = new ShooterIntake(shooter);
    shooterFeedCommand = new Feed(shooter, InOutDirection.out);
    shootSequenceCommand = new SequentialCommandGroup(new ParallelRaceGroup(
        angleMaxCommand,
        shooterShootCommand,
        new WaitCommand(1.5)),
        new ParallelRaceGroup(
            shooterFeedCommand,
            new WaitCommand(0.5)));

    autoIntakeCommand = new AutoIntakeCommand(drivebase, shooter, intake);

    registerPathPlannerNamedCommands();

    configureBindings();

    Command fieldRelative = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverYoke.getX(), OperatorConstants.X_DEADBAND),
        () -> MathUtil.applyDeadband(-driverYoke.getY(), OperatorConstants.Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverYoke.getZ(), OperatorConstants.Z_DEADBAND), true);

    Command robotRelative = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverYoke.getX(), OperatorConstants.X_DEADBAND),
        () -> MathUtil.applyDeadband(-driverYoke.getY(), OperatorConstants.Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverYoke.getZ(), OperatorConstants.Z_DEADBAND),
        false);

    m_teleopChooser.addOption("Robot Relative", robotRelative);
    m_teleopChooser.setDefaultOption("Field Relative", fieldRelative);

    m_autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto", m_autoChooser);
    SmartDashboard.putData("TeleOp", m_teleopChooser);

    burnFlash();
    clearStickyFaults();
  }

  public void registerPathPlannerNamedCommands() {
    NamedCommands.registerCommand("ANGLE_MAX", angleMaxCommand);
    NamedCommands.registerCommand("ANGLE_MIN", angleMinCommand);

    NamedCommands.registerCommand("SHOOTER_SHOOT", shooterShootCommand);
    NamedCommands.registerCommand("SHOOTER_FEED", shooterFeedCommand);
    NamedCommands.registerCommand("SHOOTER_STOP_ALL", new InstantCommand(shooter::stopAll));

    NamedCommands.registerCommand("INTAKE_IN_FRONT", intakeInCommand);
    NamedCommands.registerCommand("INTAKE_IN_SHOOTER", shooterIntakeCommand);
    NamedCommands.registerCommand("INTAKE_STOP", new InstantCommand(intake::stop));
  }

  public void homeAngle() {
    angle.bootOffset();
  }

  public boolean elevatorBottomedOut() {
    return elevator.isBottomedOut();
  }

  public void resetElevatorEncoder() {
    elevator.resetElevatorEncoder();
  }

  public void zeroGyro() {
    drivebase.zeroGyro();
  }

  public void clearStickyFaults() {
    pdh.clearStickyFaults();
    intake.clearStickyFaults();
    elevator.clearStickyFaults();
    angle.clearStickyFaults();
    shooter.clearStickyFaults();
  }

  public void burnFlash() {
    angle.burnFlash();
    elevator.burnFlash();
    intake.burnFlash();
    shooter.burnFlash();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate,
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // Yoke

    driverYoke.button(12).onTrue(zeroGyroCommand);

    driverYoke.button(4).whileTrue(autoIntakeCommand);

    driverYoke.button(1).whileTrue(shooterShootCommand);
    driverYoke.button(2).whileTrue(shooterFeedCommand);
    driverYoke.button(3).whileTrue(new ParallelRaceGroup(shooterIntakeCommand, intakeInCommand));
    driverYoke.button(6).whileTrue(shooterShootAmpCommand);
    driverYoke.button(10).whileTrue(new InstantCommand(drivebase::lock));

    // new JoystickButton(driverYoke, 5).onTrue(elevatorHangCommand);

    // Controller

    driverXbox.leftStick().onTrue(new InstantCommand(elevator::resetElevatorEncoder));

    driverXbox.pov(0).onTrue(elevatorMaxCommand);
    driverXbox.pov(270).onTrue(elevatorHalfCommand);
    driverXbox.pov(180).onTrue(elevatorMinCommand);

    driverXbox.y().onTrue(angleMaxCommand);
    driverXbox.a().onTrue(angleMinCommand);
    driverXbox.x().onTrue(angleHalfCommand);
    driverXbox.b().onTrue(anglePodiumCommand);

    // new JoystickButton(driverXbox,
    // XboxController.Button.kLeftBumper.value).whileTrue(intakeInCommand);
    driverXbox.rightBumper().whileTrue(intakeOutCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public Command getSmartAutonomous() {
    return new SequentialCommandGroup(
        // Preloaded note
        shootSequenceCommand,

        // LL intake, pathfind, shoot loop
        new RepeatCommand(
            new SequentialCommandGroup(
                turnAroundCommand,
                new ParallelRaceGroup(angleMinCommand, autoIntakeCommand),
                new ParallelRaceGroup(angleMaxCommand, drivebase.pathfindToSpeaker(SpeakerPosition.middle)),
                shootSequenceCommand)));
  }

  public void setDriveMode() {
    drivebase.setDefaultCommand(m_teleopChooser.getSelected());
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
    elevator.lock(brake);
    angle.lock(brake);
  }
}
