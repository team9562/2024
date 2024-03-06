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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.subsystems.angle.HomeAngle;
import frc.robot.commands.subsystems.angle.RotateSetpoint;
import frc.robot.commands.subsystems.angle.RotateShooter;
import frc.robot.commands.subsystems.elevator.HomeElevator;
import frc.robot.commands.subsystems.elevator.MoveElevator;
import frc.robot.commands.subsystems.elevator.MoveSetpoint;
import frc.robot.commands.subsystems.intake.Intake;
import frc.robot.commands.subsystems.shooter.Feed;
import frc.robot.commands.subsystems.shooter.Shoot;
import frc.robot.subsystems.AngleSubystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.types.AngleSetpoint;
import frc.robot.types.ElevatorSetpoint;
import frc.robot.types.InOutDirection;
import frc.robot.types.UpDownDirection;
import frc.robot.util.Utility;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

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
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final AngleSubystem angle = new AngleSubystem();

  private final SendableChooser<Command> autoChooser;

  private final PowerDistribution pdh = new PowerDistribution();

  Joystick driverYoke = new Joystick(1);
  XboxController driverXbox = new XboxController(0);

  private final SendableChooser<Command> m_commandChooser = new SendableChooser<>();
  private final SendableChooser<String> m_pathChooser = new SendableChooser<>();

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

    m_commandChooser.addOption("Yoke TeleOp [Robot]", yokeTeleopRobotRelative);
    m_commandChooser.setDefaultOption("Yoke TeleOp [Field]", yokeTeleopFieldRelative);

    getPaths();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    SmartDashboard.putData(m_commandChooser);
    SmartDashboard.putData(m_pathChooser);

    drivebase.setDefaultCommand(m_commandChooser.getSelected());
  }

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
    new JoystickButton(driverYoke, 12).onTrue((new InstantCommand(drivebase::zeroGyro)));

    new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value)
        .onTrue(new MoveSetpoint(elevator, angle, ElevatorSetpoint.min).andThen(new RotateSetpoint(angle, AngleSetpoint.min)).andThen(new Intake(intake, InOutDirection.in)).alongWith(new Shoot(shooter, InOutDirection.in))).onFalse(new InstantCommand(intake::stop).andThen(shooter::stopAll, shooter));
    new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value)
        .whileTrue(new Intake(intake, InOutDirection.out));

    new POVButton(driverXbox, 0).whileTrue(new MoveElevator(elevator, UpDownDirection.up, 0.25));
    new POVButton(driverXbox, 180).whileTrue(new MoveElevator(elevator, UpDownDirection.down, 0.25));
    new POVButton(driverXbox, 270).onTrue(new HomeElevator(elevator, angle));
    new POVButton(driverXbox, 90).onTrue(new MoveSetpoint(elevator, angle, ElevatorSetpoint.max));

    new JoystickButton(driverYoke, 1).whileTrue(new Shoot(shooter, InOutDirection.out));
    new JoystickButton(driverYoke, 2).whileTrue(new Feed(shooter, InOutDirection.out));

    new JoystickButton(driverYoke, 3).whileTrue(new Shoot(shooter, InOutDirection.in));
    new JoystickButton(driverYoke, 6).onTrue(new InstantCommand(shooter::shootAmpMaxSpeed))
        .onFalse(new InstantCommand(shooter::stopShooters));

    new JoystickButton(driverXbox, XboxController.Button.kY.value)
        .whileTrue(new RotateShooter(angle, UpDownDirection.up, 0.25));
    new JoystickButton(driverXbox, XboxController.Button.kA.value)
        .whileTrue(new RotateShooter(angle, UpDownDirection.down, 0.25));
        new JoystickButton(driverXbox, XboxController.Button.kX.value).onTrue(new HomeAngle(angle));
    new JoystickButton(driverXbox, XboxController.Button.kB.value).onTrue(new RotateSetpoint(angle, AngleSetpoint.max));

    // new JoystickButton(driverXbox, 3).onTrue(new
    // InstantCommand(drivebase::addFakeVisionReading));

    // new JoystickButton(driverXbox,
    // 2).whileTrue(
    // Commands.deferredProxy(() -> drivebase.driveToPose(
    // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
    // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new
    // InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    PathPlannerPath testPath = PathPlannerPath.fromPathFile(m_pathChooser.getSelected());
    return AutoBuilder.followPath(testPath);
  }

  public void setDriveMode() {
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
