package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.ApriltagConstants;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

public final class ApriltagAlgorithms {
  public static boolean isUsable(PhotonTrackedTarget target) {
    return ApriltagConstants.FIELD_LAYOUT.getTagPose(target.getFiducialId()).isPresent()
        && target.getPoseAmbiguity() < ApriltagConstants.MAXIMUM_AMBIGUITY
        && target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm()
            < ApriltagConstants.SINGLE_TAG_CUTOFF_METER;
  }

  public static Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {
    int numTags = 0;
    double totalDistance = 0;
    for (PhotonTrackedTarget target : targets) {
      var tagPose = ApriltagConstants.FIELD_LAYOUT.getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) continue;

      numTags++;
      totalDistance +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    double avgDistance = totalDistance / numTags;

    // Decrease std deviations further if more than one target is available
    Matrix<N3, N1> stdDevs =
        numTags == 1 ? ApriltagConstants.SINGLE_TAG_STD_DEVS : ApriltagConstants.MULTI_TAG_STD_DEVS;

    // Increase std devs based on average distance
    stdDevs = stdDevs.times(1 + (avgDistance * avgDistance / 30.0));

    return stdDevs;
  }
}
