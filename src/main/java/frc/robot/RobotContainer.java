package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.angle.RotateSetpointPercentage;
import frc.robot.commands.auto.FourNoteH213;
import frc.robot.commands.auto.MessUpC;
import frc.robot.commands.auto.ThreeNoteH21;
import frc.robot.commands.auto.ThreeNoteH23;
import frc.robot.commands.auto.TwoNoteC1;
import frc.robot.commands.auto.TwoNoteC4;
import frc.robot.commands.auto.TwoNoteC5;
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
import frc.robot.subsystems.NotesVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.types.AngleSetpoint;
import frc.robot.types.ElevatorSetpoint;
import frc.robot.types.InOutDirection;
import frc.robot.types.SpeakerPosition;
import java.io.File;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final AngleSubystem angle = new AngleSubystem();
  private final NotesVisionSubsystem notesVision = new NotesVisionSubsystem();
  // private final AprilTagsVisionSubsystem aprilTagsVision = new
  // AprilTagsVisionSubsystem(); // unused

  private final PowerDistribution pdh = new PowerDistribution();

  CommandJoystick driverYoke = new CommandJoystick(1);
  CommandXboxController driverXbox = new CommandXboxController(0);

  private final SendableChooser<Command> m_teleopChooser = new SendableChooser<>();
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final SendableChooser<SpeakerPosition> m_startPositionChooser = new SendableChooser<>();

  private final Command zeroGyroCommand = new InstantCommand(drivebase::zeroGyro);
  private final Command turnAroundCommand = new TurnAroundCommand(drivebase);
  private final Command autoIntakeCommand = new AutoIntakeCommand(drivebase, shooter, intake, angle, notesVision);
  private final Command elevatorMinCommand = new MoveSetpoint(elevator, angle, ElevatorSetpoint.min);
  private final Command elevatorHangCommand = new MoveSetpoint(elevator, angle, ElevatorSetpoint.hang);
  private final Command elevatorHalfCommand = new MoveSetpoint(elevator, angle, ElevatorSetpoint.half);
  private final Command elevatorMaxCommand = new MoveSetpoint(elevator, angle, ElevatorSetpoint.max);
  private final Command angleMinCommand = new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.min,
      false);
  private final Command angleHalfCommand = new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.half,
      false);
  private final Command anglePodiumCommand = new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.podium,
      false);
  private final Command angleMaxCommand = new RotateSetpointPercentage(angle, elevator, intake, AngleSetpoint.max,
      false);
  private final Command shooterShootCommand = new Shoot(shooter, false);
  private final Command shooterShootAmpCommand = new ShootAmp(shooter);
  private final Command shooterIntakeCommand = new ShooterIntake(shooter);
  private final Command shooterFeedCommand = new Feed(shooter, InOutDirection.out);
  private final Command intakeInCommand = new Intake(intake, angle, InOutDirection.in);
  private final Command intakeOutCommand = new Intake(intake, angle, InOutDirection.out);

  private final Command fieldRelative = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(-driverYoke.getX(), OperatorConstants.X_DEADBAND),
      () -> MathUtil.applyDeadband(-driverYoke.getY(), OperatorConstants.Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverYoke.getZ(), OperatorConstants.Z_DEADBAND), true);
  private final Command robotRelative = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(-driverYoke.getX(), OperatorConstants.X_DEADBAND),
      () -> MathUtil.applyDeadband(-driverYoke.getY(), OperatorConstants.Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverYoke.getZ(), OperatorConstants.Z_DEADBAND),
      false);

  public RobotContainer() {
    configureBindings();
    populateChoosers();
    burnFlash();
    clearStickyFaults();
  }

  private void populateChoosers() {
    m_teleopChooser.addOption("Robot Relative", robotRelative);
    m_teleopChooser.setDefaultOption("Field Relative", fieldRelative);

    m_autoChooser.addOption("Mess Up Center Notes",
        new MessUpC(angle, shooter, elevator, intake, drivebase, () -> {
          return m_startPositionChooser.getSelected() == SpeakerPosition.blueSourceSide
              || m_startPositionChooser.getSelected() == SpeakerPosition.redSourceSide;
        }));
    m_autoChooser.addOption("4 Note H231 - Speaker Middle",
        new FourNoteH213(angle, shooter, elevator, intake, drivebase, notesVision));
    m_autoChooser.addOption("2 Note C5 - Speaker Source",
        new TwoNoteC5(angle, shooter, elevator, intake, drivebase, notesVision));
    m_autoChooser.addOption("2 Note C4 - Speaker Source",
        new TwoNoteC4(angle, shooter, elevator, intake, drivebase, notesVision));
    m_autoChooser.addOption("2 Note C1 - Speaker Amp",
        new TwoNoteC1(angle, shooter, elevator, intake, drivebase, notesVision));
    m_autoChooser.addOption("3 Note H23 - Speaker Middle",
        new ThreeNoteH23(angle, shooter, elevator, intake, drivebase, notesVision));
    m_autoChooser.setDefaultOption("3 Note H21 - Speaker Middle",
        new ThreeNoteH21(angle, shooter, elevator, intake, drivebase, notesVision));

    m_startPositionChooser.addOption("[BLUE] Speaker Amp Side", SpeakerPosition.blueAmpSide);
    m_startPositionChooser.addOption("[BLUE] Speaker Source Side", SpeakerPosition.blueSourceSide);
    m_startPositionChooser.setDefaultOption("[BLUE] Speaker Middle", SpeakerPosition.blueMiddle);

    m_startPositionChooser.addOption("[RED] Speaker Amp Side", SpeakerPosition.redAmpSide);
    m_startPositionChooser.addOption("[RED] Speaker Source Side", SpeakerPosition.redSourceSide);
    m_startPositionChooser.addOption("[RED] Speaker Middle", SpeakerPosition.redMiddle);

    SmartDashboard.putData("Auto", m_autoChooser);
    SmartDashboard.putData("TeleOp", m_teleopChooser);
    SmartDashboard.putData("Starting Position", m_startPositionChooser);
  }

  public void setStartPositionOdometry() {
    drivebase.resetOdometry(m_startPositionChooser.getSelected().pose);
  }

  public void homeAngle() {
    angle.homeAngle();
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

    driverYoke.button(8).onTrue(turnAroundCommand);

    driverYoke.button(3).whileTrue(new ParallelRaceGroup(shooterIntakeCommand, intakeInCommand));
    driverYoke.button(6).whileTrue(shooterShootAmpCommand);
    driverYoke.button(10).whileTrue(new InstantCommand(drivebase::lock));

    driverYoke.button(5).onTrue(elevatorHangCommand);

    // Controller

    driverXbox.leftStick().onTrue(new InstantCommand(elevator::resetElevatorEncoder));

    // driverXbox.rightStick().onTrue(new InstantCommand(angle::resetRelativeEncoder)); // TODO: temporary

    driverXbox.pov(0).onTrue(elevatorMaxCommand);
    driverXbox.pov(270).onTrue(elevatorHalfCommand);
    driverXbox.pov(180).onTrue(elevatorMinCommand);

    driverXbox.y().onTrue(angleMaxCommand);
    driverXbox.a().onTrue(angleMinCommand);
    driverXbox.x().onTrue(angleHalfCommand);
    driverXbox.b().onTrue(anglePodiumCommand);

    driverXbox.rightBumper().whileTrue(intakeOutCommand);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
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
