package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ApriltagConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;

/** The subsystem for AprilTag vision processing. */
public final class Apriltag extends SubsystemBase {

  /** The singleton instance of the {@link Apriltag} subsystem. */
  private static Apriltag instance = null;

  /** Collection of all cameras on the robot. */
  private final Map<String, Camera> cameras = new HashMap<>();

  /** NetworkTable for publishing apriltag detection state. */
  private final NetworkTable table =
      NetworkTableInstance.getDefault().getTable("Robot").getSubTable("Apriltag");

  /**
   * Gets the instance of the {@link Apriltag} class.
   *
   * @return The instance of the {@link Apriltag} class.
   */
  public static Apriltag getInstance() {
    if (instance == null) {
      instance = new Apriltag();
    }
    return instance;
  }

  /** Constructor for the Apriltag subsystem. */
  private Apriltag() {
    // Initialize cameras from Constants
    for (ApriltagConstants.ApriltagCameraConfig cameraConfig : ApriltagConstants.PHOTON_CAMERAS) {
      Camera camera =
          new Camera(
              cameraConfig.getName(),
              cameraConfig.getTransform(),
              cameraConfig.getStrategy(),
              ApriltagConstants.SINGLE_TAG_STD_DEVS,
              ApriltagConstants.MULTI_TAG_STD_DEVS);

      cameras.put(camera.getName(), camera);

      if (!camera.camera.isConnected()) {
        DriverStation.reportWarning(
            "PhotonCamera " + camera.getName() + " is not connected!", false);
      }

      SmartDashboard.putBoolean(camera.getName() + " Connected", camera.camera.isConnected());
    }
  }

  /** Updates the pose estimation in the Swerve subsystem with vision measurements. */
  @Override
  public void periodic() {
    SwerveDrive swerveDrive = Swerve.getInstance().getSwerveDrive();
    Pose2d currentPose = Swerve.getInstance().getPose();

    // Track camera information for NetworkTables
    int totalTagsDetected = 0;
    boolean anyPoseUpdated = false;

    // Process all cameras
    for (Camera camera : cameras.values()) {
      SmartDashboard.putBoolean(camera.getName() + " Connected", camera.camera.isConnected());

      if (!camera.camera.isConnected()) continue;

      // Get and process vision measurements
      Optional<EstimatedRobotPose> estimatedPose = camera.getEstimatedGlobalPose(currentPose);

      // Check how many tags this camera sees
      int cameraTagCount = 0;
      for (PhotonPipelineResult result : camera.resultsList) {
        if (result.hasTargets()) {
          cameraTagCount = Math.max(cameraTagCount, result.getTargets().size());
        }
      }
      totalTagsDetected += cameraTagCount;

      // Publish camera-specific data
      NetworkTable cameraTable = table.getSubTable(camera.getName());
      cameraTable.getEntry("tagsDetected").setInteger(cameraTagCount);
      cameraTable.getEntry("isConnected").setBoolean(camera.camera.isConnected());

      if (estimatedPose.isPresent()) {
        anyPoseUpdated = true;
        EstimatedRobotPose pose = estimatedPose.get();

        // Extract targets from the most recent result
        List<PhotonTrackedTarget> targets = new ArrayList<>();
        if (!camera.resultsList.isEmpty()) {
          PhotonPipelineResult latestResult = camera.resultsList.get(0);
          if (latestResult.hasTargets()) {
            targets = latestResult.getTargets();
          }
        }

        // Add vision measurement to swerve drive
        swerveDrive.addVisionMeasurement(
            pose.estimatedPose.toPose2d(),
            pose.timestampSeconds,
            getEstimationStdDevs(pose.estimatedPose.toPose2d(), targets));

        cameraTable.getEntry("lastPoseTimestamp").setDouble(pose.timestampSeconds);
      }
    }

    // Update global vision status
    table.getEntry("totalTagsDetected").setInteger(totalTagsDetected);
    table.getEntry("visionUpdatingPose").setBoolean(anyPoseUpdated);
  }

  /**
   * Calculate a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag The ID of the AprilTag
   * @param robotOffset The offset to apply to the pose for proper robot positioning
   * @return The target pose for the robot relative to the AprilTag
   */
  public Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose = ApriltagConstants.FIELD_LAYOUT.getTagPose(aprilTag);
    if (aprilTagPose.isPresent()) {
      return aprilTagPose.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot find AprilTag " + aprilTag + " in field layout");
    }
  }

  /**
   * Get the distance of the robot from a specific AprilTag.
   *
   * @param id AprilTag ID
   * @return Distance in meters, or -1 if tag not found
   */
  public double getDistanceFromAprilTag(int id) {
    Pose2d currentPose = Swerve.getInstance().getPose();
    Optional<Pose3d> tag = ApriltagConstants.FIELD_LAYOUT.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose, pose3d.toPose2d()))
        .orElse(-1.0);
  }

  /**
   * Get tracked target from a camera for a specific AprilTag ID.
   *
   * @param id AprilTag ID
   * @param cameraName Name of the camera to check
   * @return Tracked target, or null if not found
   */
  public PhotonTrackedTarget getTargetFromId(int id, String cameraName) {
    Camera camera = cameras.get(cameraName);
    if (camera == null) return null;

    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          if (target.getFiducialId() == id) {
            return target;
          }
        }
      }
    }
    return null;
  }

  /**
   * Get all cameras configured in this subsystem.
   *
   * @return Map of camera name to Camera object
   */
  public Map<String, Camera> getCameras() {
    return cameras;
  }

  /** Class representing a camera on the robot. */
  public static class Camera {
    /** Latency alert for high camera latency */
    public final Alert latencyAlert;

    /** Camera instance for communication */
    public final PhotonCamera camera;

    /** Pose estimator for this camera */
    public final PhotonPoseEstimator poseEstimator;

    /** Standard deviation for single tag readings */
    private final Matrix<N3, N1> singleTagStdDevs;

    /** Standard deviation for multi-tag readings */
    private final Matrix<N3, N1> multiTagStdDevs;

    /** Camera transform relative to robot center */
    private final Transform3d robotToCamTransform;

    /** Current standard deviations in use */
    public Matrix<N3, N1> curStdDevs;

    /** Latest estimated robot pose */
    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    /** Results cache to avoid unnecessary queries */
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();

    /** Last timestamp when camera was read */
    private double lastReadTimestamp = 0;

    /** Count of times that the odometry thinks we're far from the April Tag. */
    private int longDistancePoseEstimationCount = 0;

    /**
     * Constructs a camera configuration.
     *
     * @param name Camera name in PhotonVision
     * @param transform Camera transform relative to robot
     * @param strategy Pose estimation strategy
     * @param singleTagStdDevs Standard deviations for single tag detection
     * @param multiTagStdDevs Standard deviations for multi-tag detection
     */
    public Camera(
        String name,
        Transform3d transform,
        PoseStrategy strategy,
        Matrix<N3, N1> singleTagStdDevs,
        Matrix<N3, N1> multiTagStdDevs) {
      this.latencyAlert =
          new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);
      this.camera = new PhotonCamera(name);
      this.robotToCamTransform = transform;
      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevs;
      this.curStdDevs = singleTagStdDevs;

      this.poseEstimator =
          new PhotonPoseEstimator(ApriltagConstants.FIELD_LAYOUT, strategy, robotToCamTransform);
      this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * Get the camera name.
     *
     * @return Camera name
     */
    public String getName() {
      return camera.getName();
    }

    /**
     * Get estimated global robot pose from this camera.
     *
     * @param currentPose Current robot pose for filtering
     * @return Estimated robot pose if available
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d currentPose) {
      updateUnreadResults();

      if (estimatedRobotPose.isPresent()) {
        // Filter poses that are too far from current position
        double distance =
            PhotonUtils.getDistanceToPose(
                currentPose, estimatedRobotPose.get().estimatedPose.toPose2d());

        if (distance > 1.0) {
          longDistancePoseEstimationCount++;
          if (longDistancePoseEstimationCount < 10) {
            return Optional.empty();
          }
        } else {
          longDistancePoseEstimationCount = 0;
        }
      }

      return estimatedRobotPose;
    }

    /** Update cache of unread results from camera. */
    private void updateUnreadResults() {
      double mostRecentTimestamp =
          resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      double currentTimestamp = NetworkTablesJNI.now() / 1000000.0;
      double debounceTime = ApriltagConstants.CAMERA_DEBOUNCE_TIME;

      for (PhotonPipelineResult result : resultsList) {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }

      if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime))
          && (currentTimestamp - lastReadTimestamp) >= debounceTime) {

        // Get all unread results
        List<PhotonPipelineResult> newResults = camera.getAllUnreadResults();

        // Filter results that have targets and add them to the list
        for (PhotonPipelineResult result : newResults) {
          if (result.hasTargets()) {
            resultsList.add(result);
          }
        }

        // Sort results by timestamp (newest first)
        if (!resultsList.isEmpty()) {
          resultsList.sort(
              (a, b) -> Double.compare(b.getTimestampSeconds(), a.getTimestampSeconds()));

          // Trim the list if it exceeds the maximum cache size
          while (resultsList.size() > ApriltagConstants.MAX_CAMERA_RESULTS) {
            resultsList.remove(resultsList.size() - 1);
          }
        }

        lastReadTimestamp = currentTimestamp;

        if (!resultsList.isEmpty()) {
          updateEstimatedGlobalPose();
        }
      }
    }

    /** Update the estimated global pose from camera results. */
    private void updateEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();

      for (var result : resultsList) {
        // Filter by ambiguity if needed
        if (result.hasTargets()) {
          double bestAmbiguity = 1.0;
          for (PhotonTrackedTarget target : result.getTargets()) {
            double ambiguity = target.getPoseAmbiguity();
            if (ambiguity != -1 && ambiguity < bestAmbiguity) {
              bestAmbiguity = ambiguity;
            }
          }

          // Skip if ambiguity is too high
          if (bestAmbiguity > ApriltagConstants.MAXIMUM_AMBIGUITY) {
            continue;
          }
        }

        visionEst = poseEstimator.update(result);

        if (visionEst.isPresent()) {
          updateEstimationStdDevs(visionEst, result.getTargets());
          break; // Use first good result
        }
      }

      estimatedRobotPose = visionEst;
    }

    /**
     * Update standard deviations based on vision results.
     *
     * @param estimatedPose Estimated pose
     * @param targets Detected targets
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

      if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;
        return;
      }

      // Start heuristic calculation
      var estStdDevs = singleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Calculate how many tags we found and average distance
      for (var target : targets) {
        var tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
        if (tagPose.isEmpty()) {
          continue;
        }
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible
        curStdDevs = singleTagStdDevs;
      } else {
        // One or more tags visible
        avgDist /= numTags;

        // Use multi-tag std devs if multiple targets are visible
        if (numTags > 1) {
          estStdDevs = multiTagStdDevs;
        }

        // Increase std devs based on distance
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * The standard deviations of the estimated pose from {@link
   * Camera#getEstimatedGlobalPose(Pose2d)}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets The targets used in the calc for the pose.
   * @return The calculated standard deviations. Or empty if not suitable for estimation.
   */
  public static Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {
    int numTags = 0;
    double avgDistance = 0;
    for (PhotonTrackedTarget target : targets) {
      var tagPose = ApriltagConstants.FIELD_LAYOUT.getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) continue;

      numTags++;
      avgDistance +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    Matrix<N3, N1> stdDevs = ApriltagConstants.SINGLE_TAG_STD_DEVS;
    if (numTags == 0) {
      return stdDevs;
    }

    avgDistance /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) {
      stdDevs = ApriltagConstants.MULTI_TAG_STD_DEVS;
    }

    // Increase std devs based on average distance
    if (numTags == 1 && avgDistance > ApriltagConstants.SINGLE_TAG_CUTOFF_METER) {
      // Too far for only one tag, throw away
      stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      stdDevs = stdDevs.times(1 + (avgDistance * avgDistance / 30.0));
    }

    return stdDevs;
  }
}
