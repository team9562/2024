// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.auto.LeaveLine;
import frc.robot.commands.auto.SpeakerStart;
import frc.robot.commands.subsystems.angle.HomeAngle;
import frc.robot.commands.subsystems.angle.RotateSetpoint;
import frc.robot.commands.subsystems.elevator.MoveSetpoint;
import frc.robot.commands.subsystems.intake.Intake;
import frc.robot.commands.subsystems.shooter.Feed;
import frc.robot.commands.subsystems.shooter.Shoot;
import frc.robot.commands.subsystems.shooter.ShootAmp;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.types.AngleSetpoint;
import frc.robot.types.ElevatorSetpoint;
import frc.robot.types.InOutDirection;
import frc.robot.util.Utility;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.Rev2mDistanceSensor;
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

  private final SendableChooser<Command> autoChooser;

  private final PowerDistribution pdh = new PowerDistribution();

  Joystick driverYoke = new Joystick(1);
  XboxController driverXbox = new XboxController(0);

  private final SendableChooser<Command> m_commandChooser = new SendableChooser<>();
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final SendableChooser<String> m_pathChooser = new SendableChooser<>();

  private final Command zeroGyroCommand = new InstantCommand(drivebase::zeroGyro);
  // private final Command homeElevatorCommand = new HomeElevator(elevator,
  // angle);
  private final Command elevatorMinCommand = new MoveSetpoint(elevator, angle, ElevatorSetpoint.min);
  private final Command elevatorHangCommand = new MoveSetpoint(elevator, angle, ElevatorSetpoint.hang);
  private final Command elevatorHalfCommand = new MoveSetpoint(elevator, angle, ElevatorSetpoint.half);
  private final Command elevatorMaxCommand = new MoveSetpoint(elevator, angle, ElevatorSetpoint.max);
  private final Command homeAngleCommand = new HomeAngle(angle);
  private final Command angleMinCommand = new RotateSetpoint(angle, elevator, intake, AngleSetpoint.min);//.withTimeout(1);
  private final Command angleHalfCommand = new RotateSetpoint(angle, elevator, intake, AngleSetpoint.half);//.withTimeout(1);
  private final Command angleMaxCommand = new RotateSetpoint(angle, elevator, intake, AngleSetpoint.max);//.withTimeout(1);
  private final Command shooterShootCommand;
  private final Command shooterShootAmpCommand;
  private final Command shooterIntakeCommand;
  private final Command shooterFeedCommand;
  private final Command intakeInCommand = new Intake(intake, InOutDirection.in);
  private final Command intakeOutCommand = new Intake(intake, InOutDirection.out);
  private final Command fillerAutonLeaveLine = new LeaveLine(drivebase);
  private final Command fillerSpeakerInit;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Rev2mDistanceSensor distanceSensor) {
    shooter = new ShooterSubsystem(distanceSensor);

    shooterShootCommand = new Shoot(shooter, InOutDirection.out);
    shooterShootAmpCommand = new ShootAmp(shooter);
    shooterIntakeCommand = new Shoot(shooter, InOutDirection.in);
    shooterFeedCommand = new Feed(shooter, InOutDirection.out);

    fillerSpeakerInit = new SpeakerStart(drivebase, angle, shooter, elevator, intake);

    configureBindings();

    Command fieldRelative = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverYoke.getX(), OperatorConstants.X_DEADBAND),
        () -> MathUtil.applyDeadband(driverYoke.getY(), OperatorConstants.Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverYoke.getZ(), OperatorConstants.Z_DEADBAND), true);

    Command robotRelative = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverYoke.getX(), OperatorConstants.X_DEADBAND),
        () -> MathUtil.applyDeadband(driverYoke.getY(), OperatorConstants.Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverYoke.getZ(), OperatorConstants.Z_DEADBAND), false);

    m_commandChooser.addOption("Robot Relative", robotRelative);
    m_commandChooser.setDefaultOption("Field Relative", fieldRelative);

    m_autoChooser.addOption("Leave Line", fillerAutonLeaveLine);
    m_autoChooser.setDefaultOption("Speaker", fillerSpeakerInit);

    drivebase.setDefaultCommand(m_commandChooser.getSelected());

    getPaths();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Filler Auton", m_autoChooser);
    SmartDashboard.putData("TeleOp", m_commandChooser);
    SmartDashboard.putData("Path", m_pathChooser);

  }

  public void homeAngle() {
    homeAngleCommand.initialize();

    boolean homed = false;

    while (!homed) {
      homeAngleCommand.execute();

      homed = homeAngleCommand.isFinished();
    }

    homeAngleCommand.end(false);
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

  // public void homeElevator() {
  // homeElevatorCommand.initialize();

  // boolean homed = false;

  // while (!homed) {
  // homeElevatorCommand.execute();

  // homed = homeElevatorCommand.isFinished();
  // }

  // homeElevatorCommand.end(false);
  // }

  private void getPaths() {
    File[] pathFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();

    boolean firstIteration = true;

    for (File pathFile : pathFiles) {
      if (firstIteration) {
        firstIteration = false;
        continue;
      }

      m_pathChooser.addOption(Utility.stripFileExtension(pathFile.getName()),
          Utility.stripFileExtension(pathFile.getName()));
    }

    m_pathChooser.setDefaultOption(Utility.stripFileExtension(pathFiles[0].getName()),
        Utility.stripFileExtension(pathFiles[0].getName()));
  }

  public void clearStickyFaults() {
    pdh.clearStickyFaults();
    intake.clearStickyFaults();
    elevator.clearStickyFaults();
    angle.clearStickyFaults();
    shooter.clearStickyFaults();
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

    new JoystickButton(driverYoke, 12).onTrue(zeroGyroCommand);

    new JoystickButton(driverYoke, 1).whileTrue(shooterShootCommand);
    new JoystickButton(driverYoke, 2).whileTrue(shooterFeedCommand);

    new JoystickButton(driverYoke, 3).whileTrue(shooterIntakeCommand.alongWith(intakeInCommand));
    new JoystickButton(driverYoke, 6).whileTrue(shooterShootAmpCommand);

    new JoystickButton(driverYoke, 5).onTrue(elevatorHangCommand);

    // Controller

    new JoystickButton(driverXbox, XboxController.Button.kRightStick.value).onTrue(homeAngleCommand);
    new JoystickButton(driverXbox, XboxController.Button.kLeftStick.value).onTrue(new InstantCommand(elevator::resetElevatorEncoder));
    // new JoystickButton(driverXbox,
    // XboxController.Button.kLeftStick.value).onTrue(homeElevatorCommand);

    new POVButton(driverXbox, 0).onTrue(elevatorMaxCommand);
    new POVButton(driverXbox, 270).onTrue(elevatorHalfCommand);
    new POVButton(driverXbox, 180).onTrue(elevatorMinCommand);

    new JoystickButton(driverXbox, XboxController.Button.kY.value).onTrue(angleMaxCommand);
    new JoystickButton(driverXbox, XboxController.Button.kX.value).onTrue(angleHalfCommand);
    new JoystickButton(driverXbox, XboxController.Button.kA.value).onTrue(angleMinCommand);

    // new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value).whileTrue(intakeInCommand);
    new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value).whileTrue(intakeOutCommand);
  }

  public SequentialCommandGroup getAutoSequence() {
    PathPlannerPath path = PathPlannerPath.fromPathFile(m_pathChooser.getSelected());
    drivebase.resetOdometry(path.getPathPoses().get(0));

    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        angleMaxCommand,
        shooterShootCommand,
        new SequentialCommandGroup(
          new WaitCommand(3),
          shooterFeedCommand.withTimeout(1),
          new WaitCommand(1),
          new InstantCommand(shooter::stopAll)
        )
      ),
      
      AutoBuilder.followPath(path)
    );

    // return new SequentialCommandGroup(
    //   elevatorMaxCommand.withTimeout(3),
    //   shooterShootAmpCommand.withTimeout(3),
    //   shooterFeedCommand.withTimeout(2),
    //   AutoBuilder.followPath(path)
    // );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile(m_pathChooser.getSelected());
    drivebase.resetOdometry(path.getPathPoses().get(0));
    return AutoBuilder.followPath(path);
  }

  public void setDriveMode() {
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
    elevator.lock(brake);
    angle.lock(brake);
  }
}
