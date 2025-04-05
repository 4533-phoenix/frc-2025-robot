package frc.robot.helpers;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** Apriltag camera configuration. */
public final record PhotonConfig(
    /** Name of the camera. */
    String name,

    /** Transform of the camera. */
    Transform3d transform,

    /** Strategy of the camera. */
    PoseStrategy strategy) {}
