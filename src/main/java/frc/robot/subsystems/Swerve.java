package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.helpers.POI;
import java.io.File;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Manages the robot's swerve drive system, providing control over movement, autonomous path
 * following, and odometry tracking. This subsystem handles both autonomous and teleoperated drive
 * control, integrating with PathPlanner for advanced autonomous capabilities.
 *
 * <p>Features include:
 *
 * <ul>
 *   <li>Field-oriented drive control
 *   <li>Autonomous path following and path finding
 *   <li>Odometry tracking and pose estimation
 *   <li>Wheel locking for stability
 * </ul>
 *
 * <p>Uses YAGSL (Yet Another Generic Swerve Library) for underlying swerve drive implementation and
 * PathPlanner for autonomous navigation.
 */
public class Swerve extends SubsystemBase {
  private static Swerve instance;
  private SwerveDrive drivebase;

  /**
   * Returns the singleton instance of the Swerve subsystem. Creates a new instance if one does not
   * exist.
   *
   * @return the Swerve subsystem instance
   */
  public static Swerve getInstance() {
    if (instance == null) {
      instance = new Swerve();
    }
    return instance;
  }

  /**
   * Creates a new Swerve subsystem that manages drive control, path following, and odometry.
   * Initializes the swerve drive with high telemetry verbosity and configures various drive
   * parameters.
   *
   * @throws RuntimeException if swerve drive creation fails
   */
  public Swerve() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      drivebase =
          new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
              .createSwerveDrive(
                  RobotConstants.MAX_SPEED.in(MetersPerSecond),
                  new Pose2d(
                      new Translation2d(Meter.of(8.774), Meter.of(4.026)), getAllianceRotation()));
    } catch (Exception e) {
      throw new RuntimeException("Failed to create swerve drive", e);
    }

    drivebase.setHeadingCorrection(true);
    drivebase.setAngularVelocityCompensation(true, true, 0.1);
    drivebase.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    drivebase.setModuleEncoderAutoSynchronize(true, 1);
    drivebase.setChassisDiscretization(true, true, 0.02);
    drivebase.useExternalFeedbackSensor();

    setupPathPlanner();
  }

  /**
   * Returns the rotation to use based on the alliance color. 0 degrees for blue alliance, 180
   * degrees for red alliance.
   *
   * @return Rotation2d set to 0 or 180 degrees based on alliance
   */
  private Rotation2d getAllianceRotation() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return Rotation2d.fromDegrees(180);
    }
    return Rotation2d.fromDegrees(0);
  }

  /**
   * Configures PathPlanner for autonomous path following. Sets up the necessary callbacks and
   * controllers for autonomous navigation, including pose estimation, odometry reset, and velocity
   * control. Also initializes path finding warm-up for better initial performance.
   */
  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedForward = true;

      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedForward) {
              drivebase.drive(
                  speedsRobotRelative,
                  drivebase.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              drivebase.setChassisSpeeds(speedsRobotRelative);
            }
          },
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == Alliance.Red;
            }
            return false;
          },
          this);
    } catch (Exception e) {
      e.printStackTrace();
    }

    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * Creates a command to drive the robot in field-oriented mode.
   *
   * <p>Example:
   *
   * <pre>{@code
   * Swerve.getInstance().driveFieldOriented(() -> new ChassisSpeeds(1.0, 0.0, 0.5));
   * }</pre>
   *
   * @param velocity a supplier that provides the desired chassis speeds in field-oriented
   *     coordinates
   * @return a command that continuously updates drive output based on supplied velocities
   * @see ChassisSpeeds
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> drivebase.driveFieldOriented(velocity.get()));
  }

  /**
   * Locks the swerve modules in an X pattern to prevent the robot from moving. Useful for
   * maintaining position or preparing for disable.
   *
   * <p>Example:
   *
   * <pre>{@code
   * Swerve.getInstance().lockWheels();
   * }</pre>
   */
  public void lockWheels() {
    drivebase.lockPose();
  }

  /**
   * Resets the robot's odometry to the center of the field (8.774m, 4.026m), with rotation based on
   * alliance (0° for blue, 180° for red). This is typically used at the start of autonomous
   * routines.
   *
   * <p>Example:
   *
   * <pre>{@code
   * Swerve.getInstance().resetOdometry();
   * }</pre>
   */
  public void resetOdometry() {
    drivebase.resetOdometry(
        new Pose2d(new Translation2d(Meter.of(8.774), Meter.of(4.026)), getAllianceRotation()));
  }

  /**
   * Retrieves the current estimated pose of the robot on the field.
   *
   * <p>Example:
   *
   * <pre>{@code
   * Pose2d currentPose = Swerve.getInstance().getPose();
   * }</pre>
   *
   * @return the current Pose2d representing the robot's position and rotation
   */
  public Pose2d getPose() {
    return drivebase.getPose();
  }

  /**
   * Resets the robot's odometry to a specific pose.
   *
   * <p>Example:
   *
   * <pre>{@code
   * Swerve.getInstance().resetOdometry(new Pose2d(new Translation2d(8.0, 4.0), Rotation2d.fromDegrees(90)));
   * }</pre>
   *
   * @param pose the Pose2d to set as the robot's current position and rotation
   */
  public void resetOdometry(Pose2d pose) {
    drivebase.resetOdometry(pose);
  }

  /**
   * Gets the current velocity of the robot.
   *
   * <p>Example:
   *
   * <pre>{@code
   * ChassisSpeeds speeds = Swerve.getInstance().getRobotVelocity();
   * }</pre>
   *
   * @return the ChassisSpeeds representing the robot's current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return drivebase.getRobotVelocity();
  }

  /**
   * Gets the underlying SwerveDrive object.
   *
   * <p>Example:
   *
   * <pre>{@code
   * SwerveDrive drive = Swerve.getInstance().getSwerveDrive();
   * }</pre>
   *
   * @return the SwerveDrive instance used by this subsystem
   */
  public SwerveDrive getSwerveDrive() {
    return drivebase;
  }

  /**
   * Creates a command to autonomously drive the robot to a specific pose using PathPlanner.
   *
   * <p>Example:
   *
   * <pre>{@code
   * Command autoDrive = Swerve.getInstance()
   *         .driveToPose(new Pose2d(new Translation2d(6.0, 3.0), Rotation2d.fromDegrees(45)));
   * }</pre>
   *
   * @param targetPose the target Pose2d to drive to
   * @return a Command that will drive the robot to the specified pose
   */
  public Command driveToPose(Pose2d targetPose) {
    // Use conservative constraints for accurate trajectory following
    PathConstraints constraints =
        new PathConstraints(
            5.0, // max velocity (m/s)
            1.5, // max acceleration (m/s²)
            2.0, // max angular velocity (rad/s)
            Degree.of(90).in(Radian) // max angular acceleration (rad/s²)
            );
    // Build the path following command with pathplanner's AutoBuilder
    Command pathCommand =
        AutoBuilder.pathfindToPose(targetPose, constraints, MetersPerSecond.of(0));

    // After path following, hold the position indefinitely using simple P controllers
    final double kPx = 7.5;
    final double kPy = 7.5;
    final double kPTheta = 1.0;
    Command holdCommand =
        run(
            () -> {
              Pose2d current = getPose();
              double errorX = targetPose.getX() - current.getX();
              double errorY = targetPose.getY() - current.getY();
              double errorTheta =
                  targetPose.getRotation().minus(current.getRotation()).getRadians();

              // Calculate velocities with P controller
              double vx = kPx * errorX;
              double vy = kPy * errorY;
              double omega = kPTheta * errorTheta;

              // Clamp x and y velocities between -1 and 1 m/s
              vx = Math.max(-1.0, Math.min(1.0, vx));
              vy = Math.max(-1.0, Math.min(1.0, vy));

              ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(vx, vy, omega);
              ChassisSpeeds robotRelativeSpeeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, current.getRotation());
              drivebase.setChassisSpeeds(robotRelativeSpeeds);
            });
    return pathCommand.andThen(holdCommand); // This command never finishes on its own
  }

  /**
   * Finds the closest point from an array of positions to the robot's current position. Uses
   * squared distance for optimization to avoid expensive square root operations.
   *
   * @param points Array of Pose2d positions to check
   * @return The closest Pose2d from the array, or null if the array is empty
   */
  public Pose2d getClosestPoint(Pose2d[] points) {
    if (points == null || points.length == 0) {
      return null;
    }

    Pose2d currentPose = getPose();
    double minDistSq = Double.MAX_VALUE;
    Pose2d closest = null;

    for (Pose2d point : points) {
      // Calculate squared distance to avoid square root operation
      double dx = currentPose.getX() - point.getX();
      double dy = currentPose.getY() - point.getY();
      double distSq = dx * dx + dy * dy;

      if (distSq < minDistSq) {
        minDistSq = distSq;
        closest = point;
      }
    }

    return closest;
  }

  /**
   * Finds the closest point from an array of field POIs to the robot's current position. Takes
   * alliance into account and optimizes calculations for frequent calls.
   *
   * @param points Array of POI objects to check
   * @return The Pose2d of the closest POI, or null if the array is empty
   */
  public Pose2d getClosestPOI(POI[] points) {
    if (points == null || points.length == 0) {
      return null;
    }

    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Pose2d currentPose = getPose();
    double minDistSq = Double.MAX_VALUE;
    int closestIndex = -1;

    // Cache current position for optimization
    final double robotX = currentPose.getX();
    final double robotY = currentPose.getY();

    for (int i = 0; i < points.length; i++) {
      Pose2d pointPose = points[i].get(alliance);

      // Calculate squared distance to avoid square root operation
      double dx = robotX - pointPose.getX();
      double dy = robotY - pointPose.getY();
      double distSq = dx * dx + dy * dy;

      if (distSq < minDistSq) {
        minDistSq = distSq;
        closestIndex = i;
      }
    }

    return closestIndex >= 0 ? points[closestIndex].get(alliance) : null;
  }

  /**
   * Finds the closest POI with a specific tag to the robot's current position. Highly optimized for
   * frequent calls in control loops.
   *
   * @param points Array of POI objects to check
   * @param tag The tag to filter by, or null to check all POIs
   * @return The Pose2d of the closest matching POI, or null if none found
   */
  public Pose2d getClosestPOIByTag(POI[] points, String tag) {
    if (points == null || points.length == 0) {
      return null;
    }

    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Pose2d currentPose = getPose();
    double minDistSq = Double.MAX_VALUE;
    int closestIndex = -1;

    // Cache current position for optimization
    final double robotX = currentPose.getX();
    final double robotY = currentPose.getY();

    for (int i = 0; i < points.length; i++) {
      // Skip POIs that don't match the requested tag
      if (tag != null && !points[i].getTag().equals(tag)) {
        continue;
      }

      Pose2d pointPose = points[i].get(alliance);

      // Calculate squared distance to avoid square root operation
      double dx = robotX - pointPose.getX();
      double dy = robotY - pointPose.getY();
      double distSq = dx * dx + dy * dy;

      if (distSq < minDistSq) {
        minDistSq = distSq;
        closestIndex = i;
      }
    }

    return closestIndex >= 0 ? points[closestIndex].get(alliance) : null;
  }

  /**
   * Creates a supplier that returns a Rotation2d pointing toward the closest POI with a specific
   * tag.
   *
   * @param points Array of POIs to target
   * @param tag The tag to filter by, or null to consider all POIs
   * @return A supplier that provides the rotation toward the closest matching POI when called
   */
  public Supplier<Rotation2d> createPointToClosestSupplier(POI[] points, String tag) {
    return () -> {
      Pose2d closestPose = getClosestPOIByTag(points, tag);
      if (closestPose == null) {
        return new Rotation2d(); // Default to 0 if no points available
      }
      return closestPose.getRotation();
    };
  }

  /**
   * For backward compatibility.
   *
   * @param points Array of POIs to target.
   * @return A supplier that provides the rotation toward the closest matching POI when called.
   */
  @Deprecated
  public Supplier<Rotation2d> createPointToClosestSupplier(POI[] points) {
    return createPointToClosestSupplier(points, null);
  }

  /** Resets the odometry if the robot has not received a global pose from the AprilTag system. */
  @Override
  public void periodic() {
    if (!Apriltag.getInstance().hasReceivedGlobalPose() && !Robot.getInstance().hasLeftDisabled()) {
      resetOdometry(new Pose2d(
        getPose().getTranslation(),
        getAllianceRotation()
      ));
    }
  }
}
