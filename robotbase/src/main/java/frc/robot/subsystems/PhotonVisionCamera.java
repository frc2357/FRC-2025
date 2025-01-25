package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Constants.PHOTON_VISION;
import frc.robot.Robot;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Controls the photon vision camera options. */
public class PhotonVisionCamera extends SubsystemBase {

  /*
   * The class for the object we use to cache our target data
   */
  private static class TargetInfo {

    public double yaw = Double.NaN;
    public double pitch = Double.NaN;
    public long timestamp = 0;
  }

  // all of these are private so we can use them in the extended classes
  // which are only extended so we can control which pipelines we are using.

  /** The actual camera object that we get everything from. */
  protected PhotonCamera m_camera;

  /** The result we fecth from PhotonLib each loop. */
  protected PhotonPipelineResult m_result;

  /**
   * The list of TargetInfo objects where we cache all of the target data.
   *
   * <p>Index 0 is the best gamepeice that we detect.
   *
   * <p>Index 1-16 are the AprilTags that are on the field.
   */
  protected final TargetInfo[] m_aprilTagInfo;

  /**
   * The pose estimator for all photon cameras.
   */
  protected static final PhotonPoseEstimator m_poseEstimator =
    new PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
      PHOTON_VISION.PRIMARY_STRATEGY,
      new Transform3d()
    );

  protected Transform3d m_robotToCameraTranform;

  protected static EstimatedRobotPose m_lastEstimatedPose;

  /** Whether or not we have connection with the camera still */
  protected boolean m_connectionLost;

  /**
   * The fiducial ID of the best target we have.
   *
   * <p>Used for methods that dont take in a fid ID but do some april tag stuff.
   */
  protected int m_bestTargetFiducialId;

  // experimental "turbo switch" that has the ability to increase FPS. Do not fiddle with it.
  @SuppressWarnings("unused")
  private static final BooleanEntry m_turboSwitch =
    NetworkTableInstance.create()
      .getBooleanTopic("/photonvision/use_new_cscore_frametime")
      .getEntry(PHOTON_VISION.ACTIVATE_TURBO_SWITCH);

  public PhotonVisionCamera(String cameraName, Transform3d cameraTransform) {
    m_camera = new PhotonCamera(cameraName);

    // 1-22 correspond to apriltag fiducial IDs, 0 is for gamepeices.
    m_aprilTagInfo = new TargetInfo[23];
    for (int i = 0; i < m_aprilTagInfo.length; i++) {
      m_aprilTagInfo[i] = new TargetInfo();
    }

    m_robotToCameraTranform = cameraTransform;
  }

  /**
   * Fetches the latest pipeline result. Dont call this unless you want to break everything.
   *
   * <p>
   *
   * <h1>YOU SHOULD NEVER CALL THIS! This is for the Robot periodic ONLY. NEVER call this method
   * outside of it. </h1>
   */
  public void updateResult() {
    if (!m_camera.isConnected() && !m_connectionLost) {
      m_connectionLost = true;
      DriverStation.reportError(
        "[" +
        m_camera.getName() +
        "]\n" +
        PHOTON_VISION.LOST_CONNECTION_ERROR_MESSAGE,
        false
      );
      return;
    }
    List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();

    if (results.isEmpty()) { // no new results, so we stop here.
      return;
    }
    m_result = results.get(0);
    for (int i = 1; i < results.size(); i++) {
      if (
        results.get(i).metadata.captureTimestampMicros >
        m_result.metadata.captureTimestampMicros
      ) { // getting the latest result, by finding the result with the highest capture timestamp.
        m_result = results.get(i);
      }
    }

    if (m_result == null || !m_result.hasTargets()) {
      return;
    }
    m_bestTargetFiducialId = m_result.getBestTarget().getFiducialId();
    if (m_bestTargetFiducialId == -1) { // -1 means the ID is not set, so its a gamepeice.
      cacheForGamepeices(m_result.targets);
    } else {
      cacheForAprilTags(m_result.targets);
    }

    if (m_connectionLost) {
      m_connectionLost = false;
      DriverStation.reportWarning(
        "[" +
        m_camera.getName() +
        "]\n" +
        PHOTON_VISION.CONNECTION_REGAINED_MESSAGE,
        false
      );
    }
    updatePose(); // update pose based on the new result
  }

  private void updatePose() {
    // update reference pose incase we want that strategy
    m_poseEstimator.setReferencePose(Robot.swerve.getFieldRelativePose2d());
    // change pose estimator settings to be correct for the current camera
    m_poseEstimator.setRobotToCameraTransform(m_robotToCameraTranform);
    m_lastEstimatedPose = m_poseEstimator.update(m_result).orElse(null);
    if (m_lastEstimatedPose != null) {
      double averageTargetDistance = 0;
      for (PhotonTrackedTarget target : m_lastEstimatedPose.targetsUsed) {
        averageTargetDistance += target
          .getBestCameraToTarget()
          .getMeasureX()
          .in(Meters);
      }
      averageTargetDistance /= m_lastEstimatedPose.targetsUsed.size();
      if ( // checks whether the estimated pose is in the field or not, and chucks it out if it isnt
        m_lastEstimatedPose.estimatedPose.getX() <
          -PHOTON_VISION.FIELD_BORDER_MARGIN.in(Meters) ||
        m_lastEstimatedPose.estimatedPose.getX() >
        FIELD_CONSTANTS.FIELD_LENGTH.in(Meters) +
        PHOTON_VISION.FIELD_BORDER_MARGIN.in(Meters) ||
        m_lastEstimatedPose.estimatedPose.getY() <
        -PHOTON_VISION.FIELD_BORDER_MARGIN.in(Meters) ||
        m_lastEstimatedPose.estimatedPose.getY() >
        FIELD_CONSTANTS.FIELD_LENGTH.in(Meters) +
        PHOTON_VISION.FIELD_BORDER_MARGIN.in(Meters) ||
        m_lastEstimatedPose.estimatedPose.getZ() <
        -PHOTON_VISION.Z_MARGIN.in(Meters) ||
        m_lastEstimatedPose.estimatedPose.getZ() >
        PHOTON_VISION.Z_MARGIN.in(Meters)
      ) {
        return;
      }

      if (
        Robot.swerve.getTranslationalVelocity().in(MetersPerSecond) >
        PHOTON_VISION.MAX_ACCEPTABLE_VELOCITY.in(MetersPerSecond)
      ) {
        return;
      }

      // the higher the confidence is, the less the estimated measurment is trusted.
      double xVelocityConf = Math.abs(
        0.2 + Robot.swerve.getXVelocity().in(MetersPerSecond)
      );
      double yVelocityConf = Math.abs(
        0.2 + Robot.swerve.getYVelocity().in(MetersPerSecond)
      );
      // we add 0.2 so that if were sitting still, it doesnt spiral into infinity.
      // its a partialy magic number, and will need tuning because of that.

      double xCoordinateConfidence =
        (Math.pow(0.8, m_lastEstimatedPose.targetsUsed.size()) *
          ((averageTargetDistance / 2) * xVelocityConf));
      double yCoordinateConfidence =
        (Math.pow(0.8, m_lastEstimatedPose.targetsUsed.size()) *
          ((averageTargetDistance / 2) * yVelocityConf));

      Robot.swerve.addVisionMeasurement(
        m_lastEstimatedPose.estimatedPose.toPose2d(),
        Utils.fpgaToCurrentTime(m_lastEstimatedPose.timestampSeconds),
        VecBuilder.fill(
          xCoordinateConfidence * PHOTON_VISION.X_STD_DEV_COEFFIECIENT,
          yCoordinateConfidence * PHOTON_VISION.Y_STD_DEV_COEFFIECIENT,
          Double.POSITIVE_INFINITY // Theta conf, should usually never change gyro from vision
        )
      );
    }
  }

  /**
   * The method to cache target data for gamepeices.
   *
   * @param targetList The list of targets that it pulls data from to cache.
   */
  private void cacheForGamepeices(List<PhotonTrackedTarget> targetList) {
    PhotonTrackedTarget bestTarget = calculateBestGamepeiceTarget(targetList);
    TargetInfo aprilTagInfo = m_aprilTagInfo[0];
    aprilTagInfo.yaw = bestTarget.getYaw();
    aprilTagInfo.pitch = bestTarget.getPitch();
    aprilTagInfo.timestamp = m_result.metadata.captureTimestampMicros;
  }

  /**
   * The method to cache target data for AprilTags.
   *
   * @param targetList The list of targets that it pulls data from to cache.
   */
  private void cacheForAprilTags(List<PhotonTrackedTarget> targetList) {
    for (PhotonTrackedTarget targetSeen : targetList) {
      int id = targetSeen.getFiducialId();
      TargetInfo aprilTagInfo = m_aprilTagInfo[id];
      aprilTagInfo.yaw = targetSeen.getYaw();
      aprilTagInfo.pitch = targetSeen.getPitch();
      aprilTagInfo.timestamp = m_result.metadata.captureTimestampMicros;
    }
  }

  /**
   * Calculates the best gamepeice in a list of PhotonTrackedTargets.
   *
   * <p>This is made to sort through gamepeices if they are next to eachother.
   *
   * @param targetList List of the targets to sort through.
   * @return The target that is in a acceptable pitch range, and is the most centered.
   */
  public PhotonTrackedTarget calculateBestGamepeiceTarget(
    List<PhotonTrackedTarget> targetList
  ) {
    double highestPitch =
      targetList.get(0).getPitch() +
      PHOTON_VISION.BEST_TARGET_PITCH_TOLERANCE.in(Degrees);
    PhotonTrackedTarget bestTarget = targetList.get(0);
    for (PhotonTrackedTarget targetSeen : targetList) {
      if (
        targetSeen.getPitch() < highestPitch &&
        Math.abs(targetSeen.getYaw()) < Math.abs(bestTarget.getYaw())
      ) {
        bestTarget = targetSeen;
      }
      // System.out.println(targetSeen.getFiducialId());
    }
    return bestTarget;
  }

  /**
   * @return Whether or not the camera is connected.
   */
  public boolean isConnected() {
    return m_connectionLost; // uses this because it will be checked every loop
  }

  /**
   * @return The current pipelines latency in milliseconds. Returns NaN if the camera is not
   *     connected.
   */
  public double getLatencyMillis() {
    return isConnected() ? m_result.metadata.getLatencyMillis() : Double.NaN;
  }

  /**
   * @return The timestamp of the latest pipeline result in seconds. Returns Double.NaN if the
   *     camera is not connected.
   */
  public double getTimestampSeconds() {
    return isConnected() ? m_result.getTimestampSeconds() : Double.NaN;
  }

  /**
   * Compares the current system time to the last cached timestamp, and sees if it is older than the
   * passsed in timeout.
   *
   * @param targetId Fiducial ID of the desired target to valid the data of. Notes have a
   *     fiducialId of 0
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return If the camera has seen the target within the timeout given
   */
  public boolean isValidTarget(int targetId, long timeoutMs) {
    long now = System.currentTimeMillis();
    long then = now - timeoutMs;

    if (targetId >= m_aprilTagInfo.length) {
      return false;
    }
    TargetInfo target = m_aprilTagInfo[targetId];

    return (
      target.timestamp > then ||
      Math.abs(target.yaw) > PHOTON_VISION.MAX_ANGLE.in(Degrees) ||
      Math.abs(target.pitch) > PHOTON_VISION.MAX_ANGLE.in(Degrees)
    );
  }

  /**
   * Sets the pipeline index to make the camera go to.
   *
   * @param index The index to make it be set to.
   */
  public void setPipeline(int index) {
    if (m_camera.getPipelineIndex() != index) {
      m_camera.setPipelineIndex(index);
    }
  }

  /**
   * Gets the pipeline index that an NT subscriber returns.
   *
   * @return The returned pipeline index number.
   */
  public int getPipeline() {
    return m_camera.getPipelineIndex();
  }

  /**
   * Gets what PhotonVision said the best target was last time it looked.
   *
   * @return The fiducial id of the best target
   */
  public int getBestTargetFiducialId() {
    return m_bestTargetFiducialId;
  }

  /**
   * @param fiducialId The fiducial ID of the target to get the yaw of.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the desired targets yaw. <strong>Will be null if the cached data was invalid.
   */
  public Angle getTargetYaw(int targetId, long timeoutMs) {
    if (isValidTarget(targetId, timeoutMs)) {
      return Units.Degrees.of(m_aprilTagInfo[targetId].yaw);
    }
    return null;
  }

  /**
   * @param fiducialIds The list of fiducial IDs to check.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the yaw of the first valid target in the list, <strong>or null if none are valid.
   */
  public Angle getTargetYaw(int[] targetIds, long timeoutMs) {
    for (int id : targetIds) {
      Angle yaw = getTargetYaw(id, timeoutMs);
      if (yaw != null) {
        return yaw;
      }
    }
    return null;
  }

  /**
   * @param id The ID of the target to get the pitch of.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the desired targets pitch, <strong>will be null if the cached data was invalid.
   */
  public Angle getTargetPitch(int targetId, long timeoutMs) {
    if (isValidTarget(targetId, timeoutMs)) {
      return Units.Degrees.of(m_aprilTagInfo[targetId].pitch);
    }
    return null;
  }

  /**
   * @param fiducialIds The list of fiducial IDs to check.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the pitch of the first valid id in the list, <strong>or null if none are valid.
   */
  public Angle getTargetPitch(int[] fiducialIds, long timeoutMs) {
    for (int id : fiducialIds) {
      Angle pitch = getTargetPitch(id, timeoutMs);
      if (pitch != null) {
        return pitch;
      }
    }
    return null;
  }

  /**
   * Gets the last estimated pose from PV's pose estimator.
   *
   * <p>Should only be used if the camera does not see more than 1 april tag, if it does, use
   * getPNPResult instead, as it is more accurate.
   *
   * @return The robots estimated pose, if it has any april tag targets. <strong>Returns null if
   *     there are no targets.</strong>
   */
  public EstimatedRobotPose getLastEstimatedPose() {
    return m_lastEstimatedPose;
  }

  /**
   * Gets the information of the multiple tag pose estimate if it exists. Be real careful with how you use this.
   *
   * <p>If the camera does not see more than 1 april tag, <strong> this will return null. </strong>
   *
   * @return The PNPResult for you to get information from.
   */
  public MultiTargetPNPResult getMultiTargetPNPResult() {
    Optional<MultiTargetPNPResult> PNPEstimate = m_result.getMultiTagResult();
    return PNPEstimate.isPresent() ? PNPEstimate.get() : null;
  }

  /**
   * @param result The MultiTargetPNPResult to take the pose from.
   * @return The Pose3d constructed from the PNPResult. RETURNS NULL IF POSE IS NOT ACCEPTABLE
   */
  public Pose3d pose3dFromPNPResult(MultiTargetPNPResult result) {
    Transform3d resultTransform = result.estimatedPose.best;
    double resultReprojectionError = result.estimatedPose.bestReprojErr;
    double resultAmbiguity = result.estimatedPose.ambiguity;

    if (resultAmbiguity > PHOTON_VISION.MAX_AMBIGUITY_TOLERANCE) {
      return null;
    }
    if (resultReprojectionError > PHOTON_VISION.MAX_REPROJECTION_ERROR_PIXELS) {
      return null;
    }
    return new Pose3d(
      resultTransform.getTranslation(),
      resultTransform.getRotation()
    );
  }

  /**
   * @param result The MultiTargetPNPResult to take the pose from.
   * @return The Pose2d constructed from the PNPResult.
   */
  public Pose2d pose2dFromPNPResult(MultiTargetPNPResult result) {
    return pose3dFromPNPResult(result).toPose2d();
  }

  /**
   * @return The number of targets seen.
   */
  public int numberOfTargetsSeen() {
    return m_result.targets.size();
  }

  /**
   * @return Whether or not the camera has a target.
   */
  public boolean hasTarget() {
    return m_result.hasTargets();
  }
}
