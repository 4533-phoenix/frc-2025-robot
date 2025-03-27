package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.helpers.CustomSwerveInput;
import frc.robot.subsystems.*;

/**
 * Main robot configuration class that binds controls and commands to subsystems. This class serves
 * as the robot's command center, managing all subsystem instances and their associated commands.
 *
 * <p>Features include:
 *
 * <ul>
 *   <li>Driver control configuration
 *   <li>Command button mappings
 *   <li>Autonomous command selection
 *   <li>Subsystem instantiation and management
 * </ul>
 *
 * <p>The class follows a centralized control pattern, with all robot behaviors defined through
 * command bindings and default commands.
 */
public class RobotContainer {
  /** Network table for robot-related bs */
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Robot");

  /** Xbox controller used for driver input. */
  private final CommandXboxController driverController =
      new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

  /** "Xbox" controller used for operator input. */
  private final CommandXboxController operatorController = new CommandXboxController(1);

  /** Main drive subsystem for robot movement. */
  private final Swerve swerveDrive = Swerve.getInstance();

  /**
   * Input stream for swerve drive control. Configures how controller inputs are processed and
   * applied to drive commands.
   */
  private final CustomSwerveInput driveInputStream =
      CustomSwerveInput.of(
              swerveDrive.getSwerveDrive(),
              () -> driverController.getLeftY() * -1,
              () -> driverController.getLeftX() * -1)
          .cubeTranslationControllerAxis(true)
          .scaleTranslation(0.75)
          .scaleTranslation(() -> driverController.rightBumper().getAsBoolean(), 0.5)
          .withControllerHeadingAxis(
              () -> driverController.getRightX() * -1, () -> driverController.getRightY() * -1)
          .cubeRotationControllerAxis(true)
          .deadband(OIConstants.DRIVER_DEADBAND)
          .allianceRelativeControl(true)
          .headingWhile(true);

  // private final SwerveInputStream driveInputStream =
  // SwerveInputStream.of(swerveDrive.getSwerveDrive(),
  // () -> driverController.getLeftY() * -1,
  // () -> driverController.getLeftX() * -1)
  // .cubeTranslationControllerAxis(true)
  // .scaleTranslation(1.0)
  // .withControllerRotationAxis(() -> driverController.getLeftX() * -1)
  // .cubeRotationControllerAxis(true)
  // .deadband(OIConstants.DRIVER_DEADBAND)
  // .allianceRelativeControl(true);

  /** Alga arm subsystem for handling alga gamepieces. */
  private final AlgaArm algaArm = AlgaArm.getInstance();

  /** Coral handler subsystem for handling coral gamepieces. */
  private final CoralManipulator coralManipulator = CoralManipulator.getInstance();

  /** Coral handler subsystem for handling coral gamepieces. */
  private final CoralArm coralArm = CoralArm.getInstance();

  /** Climb subsystem for handling climb mechanism. */
  private final Climb climb = Climb.getInstance();

  /** Apriltag subsystem for handling apriltags */
  @SuppressWarnings("unused")
  private final Apriltag apriltag = Apriltag.getInstance();

  // Add the SendableChooser for autonomous
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * Creates a new RobotContainer and initializes all robot subsystems and commands. Performs the
   * following setup:
   *
   * <ul>
   *   <li>Silences joystick warnings for unplugged controllers
   *   <li>Disables controller rumble
   *   <li>Configures button bindings for commands
   * </ul>
   */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    driverController.setRumble(RumbleType.kBothRumble, 0.0);
    operatorController.setRumble(RumbleType.kBothRumble, 0.0);

    configureBindings();
    configureAutoChooser();
  }

  /**
   * Configures button bindings for commands. Maps controller buttons to specific robot actions
   * organized by controller and function.
   */
  private void configureBindings() {
    configureDriverControls();
    configureOperatorControls();
  }

  /**
   * Configure driver controller bindings for drivetrain controls, autonomous features, and climb
   */
  private void configureDriverControls() {
    // DEFAULT COMMAND - Field-oriented drive with automatic heading
    Command driveFieldOrientedDirectAngle =
        swerveDrive.driveFieldOriented(
            driveInputStream
                .copy()
                .withHeading(
                    swerveDrive.createPointToClosestSupplier(Constants.FieldConstants.ALL_POIS))
                .headingWhile(true));
    swerveDrive.setDefaultCommand(driveFieldOrientedDirectAngle);

    // AUTONOMOUS ASSIST - Drive to closest point when holding A
    driverController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Pose2d targetPose = swerveDrive.getClosestPOI(Constants.FieldConstants.ALL_POIS);
                  if (targetPose != null) {
                    swerveDrive.driveToPose(targetPose).schedule();
                  }
                }));

    // MANUAL OVERRIDE - Use raw input without auto heading when holding left bumper
    driverController.leftBumper().whileTrue(swerveDrive.driveFieldOriented(driveInputStream));

    // CLIMBING CONTROL
    driverController.x().whileTrue(climb.climb());
  }

  /** Configure operator controller bindings for game piece and mechanism controls */
  private void configureOperatorControls() {
    // ---- ALGA ARM CONTROLS ----
    // Toggle alga arm
    operatorController.a().onTrue(algaArm.toggle());
    // Run alga intake while held
    operatorController.leftBumper().whileTrue(algaArm.runIntake());
    // Run alga drop while held
    operatorController.rightBumper().whileTrue(algaArm.runDrop());

    // ---- CORAL MANIPULATOR CONTROLS ----
    operatorController.b().onTrue(coralManipulator.ejectOrDrop().onlyIf(coralArm.onFront.negate()));

    // ---- CORAL ARM POSITION CONTROLS ----
    // Each of these stops the manipulator before moving to ensure safe operation

    // Return to zero position
    operatorController
        .back()
        .onTrue(
            Commands.runOnce(
                () ->
                    coralArm.setZero().beforeStarting(coralManipulator.instantStop()).schedule()));

    // Intake sequence - move to intake, start intake, then move to mid
    operatorController
        .povRight()
        .onTrue(
            Commands.runOnce(
                () ->
                    coralArm
                        .setIntake()
                        .beforeStarting(coralManipulator.instantStop())
                        .andThen(coralManipulator.intake())
                        .andThen(coralArm.setMid())
                        .onlyIf(coralManipulator.doesntHaveCoral)
                        .schedule()));

    // High scoring position
    operatorController
        .povUp()
        .onTrue(
            Commands.runOnce(
                () ->
                    coralArm.setHigh().beforeStarting(coralManipulator.instantStop()).schedule()));

    // Low scoring position
    operatorController
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> coralArm.setLow().beforeStarting(coralManipulator.instantStop()).schedule()));

    // Mid scoring position
    operatorController
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> coralArm.setMid().beforeStarting(coralManipulator.instantStop()).schedule()));

    // Climb position
    operatorController
        .start()
        .onTrue(
            Commands.runOnce(
                () ->
                    coralArm.setClimb().beforeStarting(coralManipulator.instantStop()).schedule()));
  }

  /** Configure the autonomous command chooser with available options. */
  private void configureAutoChooser() {
    autoChooser.setDefaultOption("None", Commands.none());
    autoChooser.addOption("Simple Backward Drive", AutoCommands.simpleBackwardDrive());
    autoChooser.addOption("Left to Reef", AutoCommands.leftToReef());
    autoChooser.addOption("Right to Reef", AutoCommands.rightToReef());
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Provides the command to run during autonomous mode. Currently returns a placeholder command
   * that prints a message, indicating no autonomous routine is configured.
   *
   * @return the command to run in autonomous mode
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Returns the NetworkTable used for debugging.
   *
   * @return the NetworkTable used for debugging
   */
  public NetworkTable getTable() {
    return table;
  }
}
