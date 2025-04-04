package frc.robot.helpers;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Point of Interest on the field Contains a pose, alliance designation, and a descriptive tag */
public final class POI {
  private final Pose2d pose;
  private final String tag;
  private final String addr;
  private Pose2d cachedRedPose; // Cache for the flipped pose

  /**
   * Creates a new POI.
   *
   * @param pose The pose of the POI.
   * @param tag The tag of the POI.
   * @param addr The address of the POI.
   */
  public POI(Pose2d pose, String tag, String addr) {
    this.pose = pose;
    this.tag = tag;
    this.addr = addr;
  }

  /**
   * Gets the pose of the POI for the given alliance.
   *
   * @param alliance The alliance to get the pose for.
   * @return The pose of the POI for the given alliance.
   */
  public Pose2d get(Alliance alliance) {
    if (alliance == Alliance.Blue) {
      return pose;
    } else {
      if (cachedRedPose == null) {
        cachedRedPose = FlippingUtil.flipFieldPose(pose);
      }
      return cachedRedPose;
    }
  }

  /**
   * Gets the descriptive tag for this POI
   *
   * @return The tag string
   */
  public String getTag() {
    return tag;
  }

  /**
   * Gets the addr for the POI
   *
   * @return The addr string
   */
  public String getAddr() {
    return addr;
  }
}
