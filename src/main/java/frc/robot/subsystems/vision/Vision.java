package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ApriltagConstants;
import frc.robot.helpers.PhotonConfig;
import frc.robot.subsystems.Swerve;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Vision extends SubsystemBase {
  private static Vision instance;

  public static class AprilTagIOInputs {
      public boolean connected;
      public Translation2d[] validCorners;
      public Translation2d[] rejectedCorners;
      public int[] validIds;
      public int[] rejectedIds;
      public PoseObservation[] validPoseObservations;
      public PoseObservation[] rejectedPoseObservations;
      public Pose3d[] validPoses;
      public Pose3d[] rejectedPoses;
      public Pose3d[] validAprilTagPoses;
      public Pose3d[] rejectedAprilTagPoses;

      // Default constructor with empty arrays
      public AprilTagIOInputs() {
          this.connected = false;
          this.validCorners = new Translation2d[0];
          this.rejectedCorners = new Translation2d[0];
          this.validIds = new int[0];
          this.rejectedIds = new int[0];
          this.validPoseObservations = new PoseObservation[0];
          this.rejectedPoseObservations = new PoseObservation[0];
          this.validPoses = new Pose3d[0];
          this.rejectedPoses = new Pose3d[0];
          this.validAprilTagPoses = new Pose3d[0];
          this.rejectedAprilTagPoses = new Pose3d[0];
      }
  }

  public static record AprilTagCamera(
      ApriltagIO io, AprilTagIOInputs inputs, PhotonConfig config, Alert disconnectedAlert) {}

  private final List<AprilTagCamera> aprilTagCameras = new ArrayList<>();

  private final Swerve swerve = Swerve.getInstance();

  private boolean hasReceivedGlobalPose = false;

  private Vision() {
    for (PhotonConfig config : ApriltagConstants.PHOTON_CAMERAS) {
      ApriltagIO io = new ApriltagIO(config);

      Alert disconnectedAlert =
          new Alert(
              "AprilTag Camera Disconnected",
              "The AprilTag camera " + config.name() + " is disconnected.",
              AlertType.kWarning);

      aprilTagCameras.add(new AprilTagCamera(io, new AprilTagIOInputs(), config, disconnectedAlert));
    }
  }
  
  /**
   * Returns the singleton instance of the Vision subsystem.
   * @return the singleton instance
   */
  public static Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  @Override
  public void periodic() {
    List<Translation2d> validCorners = new ArrayList<>();
    List<Translation2d> rejectedCorners = new ArrayList<>();

    List<Integer> validIds = new ArrayList<>();
    List<Integer> rejectedIds = new ArrayList<>();

    List<PoseObservation> validPoseObservations = new ArrayList<>();
    List<PoseObservation> rejectedPoseObservations = new ArrayList<>();

    List<Pose3d> validPoses = new ArrayList<>();
    List<Pose3d> rejectedPoses = new ArrayList<>();

    List<Pose3d> validAprilTagPoses = new ArrayList<>();
    List<Pose3d> rejectedAprilTagPoses = new ArrayList<>();

    for (AprilTagCamera cam : aprilTagCameras) {
      cam.io.updateInputs(cam.inputs);

      cam.disconnectedAlert.set(!cam.inputs.connected);
      SmartDashboard.putBoolean(cam.config.name() + " Connected", cam.inputs.connected);

      validCorners.addAll(Arrays.asList(cam.inputs.validCorners));
      rejectedCorners.addAll(Arrays.asList(cam.inputs.rejectedCorners));

      validIds.addAll(Arrays.stream(cam.inputs.validIds).boxed().toList());
      rejectedIds.addAll(Arrays.stream(cam.inputs.rejectedIds).boxed().toList());

      validPoseObservations.addAll(Arrays.asList(cam.inputs.validPoseObservations));
      rejectedPoseObservations.addAll(Arrays.asList(cam.inputs.rejectedPoseObservations));

      validPoses.addAll(Arrays.asList(cam.inputs.validPoses));
      rejectedPoses.addAll(Arrays.asList(cam.inputs.rejectedPoses));

      validAprilTagPoses.addAll(Arrays.asList(cam.inputs.validAprilTagPoses));
      rejectedAprilTagPoses.addAll(Arrays.asList(cam.inputs.rejectedAprilTagPoses));

      for (PoseObservation observation : cam.inputs.validPoseObservations) {
        swerve
            .getSwerveDrive()
            .addVisionMeasurement(
                observation.robotPose().toPose2d(), observation.timestampSeconds(), observation.stdDevs());

        if (!hasReceivedGlobalPose) {
          hasReceivedGlobalPose = true;
        }
      }
    }
  }

  public boolean hasReceivedGlobalPose() {
    return hasReceivedGlobalPose;
  }
}
