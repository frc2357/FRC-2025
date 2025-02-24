package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.PHOTON_VISION.*;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.*;

/** Controls the photon vision camera options. */
public class PhotonVisionCamera extends SubsystemBase {

  /**
   * The class for storing results with all data needed to perform the constrained PNP operation
   */
  private static class TimestampedPNPInfo {

    public PhotonPipelineResult result = null;
    public Rotation3d heading = null;
    public double headingTimestampSeconds = Double.NaN;
    public Optional<Matrix<N3, N3>> camMatrix = null;
    public Optional<Matrix<N8, N1>> distCoeefs = null;
    public Transform3d robotToCameraTransform = null;

    public TimestampedPNPInfo() {
      result = null;
      heading = null;
      headingTimestampSeconds = Double.NaN;
      camMatrix = null;
      distCoeefs = null;
      robotToCameraTransform = null;
    }

    public void replaceInfo(
      PhotonPipelineResult result,
      Rotation3d heading,
      double headingTimestamp,
      Optional<Matrix<N3, N3>> camMatrix,
      Optional<Matrix<N8, N1>> distCoeefs,
      Transform3d robotToCamTransform
    ) {
      this.result = result;
      this.heading = heading;
      this.headingTimestampSeconds = headingTimestamp;
      this.camMatrix = camMatrix;
      this.distCoeefs = distCoeefs;
      this.robotToCameraTransform = robotToCamTransform;
    }

    public void invalidateInfo() {
      result = null;
      heading = null;
      headingTimestampSeconds = Double.NaN;
      camMatrix = null;
      distCoeefs = null;
      robotToCameraTransform = null;
    }

    public static TimestampedPNPInfo[] makePNPInfoList() {
      TimestampedPNPInfo[] list =
        new TimestampedPNPInfo[PNP_INFO_STORAGE_AMOUNT];
      for (int i = 0; i < list.length; i++) {
        list[i] = new TimestampedPNPInfo();
      }
      return list;
    }
  }

  /*
   * The class for the object we use to cache our target data
   */
  // private static class TargetInfo {

  //   public double yaw = Double.NaN;
  //   public double pitch = Double.NaN;
  //   public long timestamp = 0;
  // }

  // all of these are protected so we can use them in extended classes
  // which are only extended so we can control which pipelines we are using.

  /** The actual camera object that we get everything from. */
  protected final PhotonCamera m_camera;

  /** The result we fecth from PhotonLib each loop. */
  protected PhotonPipelineResult m_result;

  /**
   * The list of TargetInfo objects where we cache all of the target data.
   *
   * <p>Index 0 is the best gamepeice that we detect.
   *
   * <p>Index 1-16 are the AprilTags that are on the field.
   */
  // protected final TargetInfo[] m_aprilTagInfo;

  /**
   * The list of {@link TimestampedPNPInfo} objects that we use to store data for the constrainedPNP calculation.
   *
   * <p> this data is used to figure out which results are the most worthwhile to use for pose est
   */
  private static final TimestampedPNPInfo[] m_pnpInfo =
    TimestampedPNPInfo.makePNPInfoList();

  /**
   * A list of all instances of this class, in order of instantiation
   */
  private static final ArrayList<PhotonVisionCamera> m_robotCameras =
    new ArrayList<PhotonVisionCamera>();

  /**
   * The pose estimator for all photon cameras.
   */
  private static final PhotonPoseEstimator m_poseEstimator =
    new PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
      PRIMARY_STRATEGY,
      new Transform3d()
    );

  protected final Transform3d m_robotToCameraTranform;

  protected final Optional<Matrix<N3, N3>> m_cameraMatrix;

  protected final Optional<Matrix<N8, N1>> m_distCoeefs;

  protected static EstimatedRobotPose m_lastEstimatedPose;

  /** Whether or not we have connection with the camera still */
  protected boolean m_connectionLost;

  /**
   * The fiducial ID of the best target we have.
   *
   * <p>Used for methods that dont take in a fid ID but do some april tag stuff.
   */
  // protected int m_bestTargetFiducialId;

  // experimental "turbo switch" that has the ability to increase FPS. Do not fiddle with it.
  @SuppressWarnings("unused")
  private static final BooleanEntry m_turboSwitch =
    NetworkTableInstance.create()
      .getBooleanTopic("/photonvision/use_new_cscore_frametime")
      .getEntry(ACTIVATE_TURBO_SWITCH);

  public PhotonVisionCamera(
    String cameraName,
    Transform3d cameraTransform,
    Matrix<N3, N3> cameraMatrix,
    Matrix<N8, N1> distCoeefs
  ) {
    m_camera = new PhotonCamera(cameraName);
    m_robotCameras.add(this);

    // 1-22 correspond to apriltag fiducial IDs, 0 is for gamepeices.
    // m_aprilTagInfo = new TargetInfo[23];
    // for (int i = 0; i < m_aprilTagInfo.length; i++) {
    //   m_aprilTagInfo[i] = new TargetInfo();
    // }

    m_robotToCameraTranform = cameraTransform;
    m_cameraMatrix = Optional.of(cameraMatrix);
    m_distCoeefs = Optional.of(distCoeefs);
  }

  /**
   * Updates all instances of {@link PhotonVisionCamera} with the latest result, and updates the pose with the updated pnpInfo.
   * <p><h1> For the top of {@link Robot#robotPeriodic() the robot periodic} only.
   */
  public static void updateAllCameras() {
    for (PhotonVisionCamera camera : m_robotCameras) {
      camera.updateResult();
    }
    for (int i = 0; i < m_pnpInfo.length; i++) {
      if (m_pnpInfo[i].result == null) {
        continue;
      }
      updatePoseFromPNPInfo(m_pnpInfo[i]);
      m_pnpInfo[i].invalidateInfo();
    }
    if (m_lastEstimatedPose != null) {
      Robot.elasticFieldManager.shooterFieldRep.setRobotPose(
        m_lastEstimatedPose.estimatedPose.toPose2d()
      );
    }
  }

  /**
   * Fetches the latest pipeline result for this instance
   */
  private void updateResult() {
    if (!m_camera.isConnected() && !m_connectionLost) {
      m_connectionLost = true;
      DriverStation.reportError(
        "[" + m_camera.getName() + "]\n" + LOST_CONNECTION_ERROR_MESSAGE,
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
    // m_bestTargetFiducialId = m_result.getBestTarget().getFiducialId();
    // if (m_bestTargetFiducialId == -1) { // -1 means the ID is not set, so its a gamepeice.
    //   cacheForGamepeices(m_result.targets);
    // } else {
    //   cacheForAprilTags(m_result.targets);
    // }

    if (m_connectionLost) {
      m_connectionLost = false;
      DriverStation.reportWarning(
        "[" + m_camera.getName() + "]\n" + CONNECTION_REGAINED_MESSAGE,
        false
      );
    }
    updatePNPInfo(m_result); // update pnpInfo with the new result
  }

  /**
   * Gets the {@link #m_poseEstimator pose estimator} ready to estimate with the provided and/or relevant information
   */
  @SuppressWarnings("incomplete-switch")
  private static void preparePoseEstimator(
    Transform3d robotToCamTransform,
    Rotation3d heading,
    double headingTimestampSeconds
  ) {
    // does whatever we need to make the strategy work
    switch (PRIMARY_STRATEGY) {
      case CONSTRAINED_SOLVEPNP, PNP_DISTANCE_TRIG_SOLVE:
        m_poseEstimator.addHeadingData(headingTimestampSeconds, heading);
        break;
      case CLOSEST_TO_REFERENCE_POSE:
        m_poseEstimator.setReferencePose(Robot.swerve.getFieldRelativePose2d());
        break;
      case CLOSEST_TO_LAST_POSE:
        m_poseEstimator.setLastPose(m_lastEstimatedPose.estimatedPose);
        break;
    }
    switch (FALLBACK_STRATEGY) {
      case CONSTRAINED_SOLVEPNP, PNP_DISTANCE_TRIG_SOLVE:
        // if our primary strategy already needs us to update the heading, we dont want to call more hardware.
        // they get upset if you call them enough, and its a waste of time.
        if (
          PRIMARY_STRATEGY == PoseStrategy.CONSTRAINED_SOLVEPNP ||
          PRIMARY_STRATEGY == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
        ) {
          break;
        }
        m_poseEstimator.addHeadingData(headingTimestampSeconds, heading);
        break;
      case CLOSEST_TO_REFERENCE_POSE:
        m_poseEstimator.setReferencePose(Robot.swerve.getFieldRelativePose2d());
        break;
      case CLOSEST_TO_LAST_POSE:
        m_poseEstimator.setLastPose(m_lastEstimatedPose.estimatedPose);
        break;
    }

    // change pose estimator settings to be correct for the current camera
    m_poseEstimator.setRobotToCameraTransform(robotToCamTransform);
  }

  private static void updatePoseFromPNPInfo(TimestampedPNPInfo pnpInfo) {
    preparePoseEstimator(
      pnpInfo.robotToCameraTransform,
      pnpInfo.heading,
      pnpInfo.headingTimestampSeconds
    );
    m_lastEstimatedPose = m_poseEstimator
      .update(
        pnpInfo.result,
        pnpInfo.camMatrix,
        pnpInfo.distCoeefs,
        POSE_EST_PARAMS
      )
      .orElse(null);
    if (m_lastEstimatedPose == null) {
      return;
    }
    double averageTargetDistance = 0;
    for (PhotonTrackedTarget target : m_lastEstimatedPose.targetsUsed) {
      averageTargetDistance += target
        .getBestCameraToTarget()
        .getMeasureX()
        .in(Meters);
    }
    averageTargetDistance /= m_lastEstimatedPose.targetsUsed.size();
    if (isPoseInField(m_lastEstimatedPose.estimatedPose)) {
      return;
    }

    if (
      Robot.swerve.getTranslationalVelocity().in(MetersPerSecond) >
      MAX_ACCEPTABLE_VELOCITY.in(MetersPerSecond)
    ) {
      return;
    }

    // the higher the confidence is, the less the estimated measurment is trusted.
    double xVelocityConf =
      0.2 + Math.abs(Robot.swerve.getXVelocity().in(MetersPerSecond));
    double yVelocityConf =
      0.2 + Math.abs(Robot.swerve.getYVelocity().in(MetersPerSecond));
    // we add 0.2 so that if were sitting still, it doesnt spiral into infinity.
    // its a partialy magic number, and will need tuning because of that.

    double xCoordinateConfidence =
      (Math.pow(0.8, m_lastEstimatedPose.targetsUsed.size()) *
        ((averageTargetDistance / 2) * xVelocityConf));
    double yCoordinateConfidence =
      (Math.pow(0.8, m_lastEstimatedPose.targetsUsed.size()) *
        ((averageTargetDistance / 2) * yVelocityConf));
    System.out.println("CURR TIME: " + Utils.getCurrentTimeSeconds());
    System.out.println(
      "FRAME TIME: " +
      Utils.fpgaToCurrentTime(m_lastEstimatedPose.timestampSeconds)
    );
    Robot.swerve.addVisionMeasurement(
      m_lastEstimatedPose.estimatedPose.toPose2d(),
      Utils.fpgaToCurrentTime(m_lastEstimatedPose.timestampSeconds),
      VecBuilder.fill(
        xCoordinateConfidence * X_STD_DEV_COEFFIECIENT,
        yCoordinateConfidence * Y_STD_DEV_COEFFIECIENT,
        Double.MAX_VALUE // Theta conf, should usually never change gyro from vision.
      )
    );
  }

  /**
   * Updates the {@link #m_pnpInfo} list with a result, and replaces the worst result, if the provided result is better.
   *
   * <p> only works for april tag only cameras.
   *
   * @param result The provided {@link PhotonPipelineResult result} to use for updating the pnpInfo list
   */
  private void updatePNPInfo(PhotonPipelineResult result) {
    // if the provided result has no targets, it has no value, so we do not want to store it
    if (!result.hasTargets()) {
      return;
    }
    double frameTimeSeconds = result.getTimestampSeconds();
    double currTimeSeconds = RobotController.getFPGATime() * 1e-6;
    // if result is older than allowed, do not store it
    if (frameTimeSeconds <= currTimeSeconds - PNP_INFO_VALID_TIME.in(Seconds)) {
      return;
    }
    Rotation3d heading = Robot.swerve.getRotation3d();
    double headingTimestampSeconds = Robot.swerve.getState().Timestamp;
    int tarCount = result.targets.size();
    int indexToReplace = -1;
    for (int i = 0; i < m_pnpInfo.length; i++) {
      // if selected info does not exist, replace it and stop the loop
      if (m_pnpInfo[i].result == null) {
        m_pnpInfo[i].replaceInfo(
            result,
            heading,
            headingTimestampSeconds,
            m_cameraMatrix,
            m_distCoeefs,
            m_robotToCameraTranform
          );
        break;
      }
      double storedInfoFrameSeconds = m_pnpInfo[i].result.getTimestampSeconds();
      // if stored data is older than allowed, invalidate it, and set it to be replaced.
      if (
        storedInfoFrameSeconds <=
        currTimeSeconds - PNP_INFO_VALID_TIME.in(Seconds)
      ) {
        m_pnpInfo[i].invalidateInfo();
        indexToReplace = i;
        continue;
      }
      // if selected info has less targets than the provided result
      else if (m_pnpInfo[i].result.targets.size() <= tarCount) {
        // if we have already selected info to replace
        if (indexToReplace > -1) {
          // if we found a worse result
          if (
            m_pnpInfo[indexToReplace].result.targets.size() >
            m_pnpInfo[i].result.targets.size()
          ) {
            // replace that one instead
            indexToReplace = i;
          }
          // if both results have the same number of targets
          else if (
            m_pnpInfo[indexToReplace].result.targets.size() ==
            m_pnpInfo[i].result.targets.size()
          ) {
            // if (i) was taken later than (indexToReplace), replace (i) instead
            if (
              m_pnpInfo[indexToReplace].result.metadata.captureTimestampMicros >
              m_pnpInfo[i].result.metadata.captureTimestampMicros
            ) {
              indexToReplace = i;
            }
            // if (i) has worse ambiguity than (indexToReplace), replace (i) instead
            else if (
              m_pnpInfo[indexToReplace].result.targets.get(0).poseAmbiguity <
              m_pnpInfo[i].result.targets.get(0).poseAmbiguity
            ) {
              indexToReplace = i;
            }
          }
        }
        // if we have not already selected something to replace
        else {
          indexToReplace = i;
        }
      }
    }
  }

  /**
   * The method to cache target data for gamepeices.
   *
   * @param targetList The list of targets that it pulls data from to cache.
   */
  // private void cacheForGamepeices(List<PhotonTrackedTarget> targetList) {
  //   PhotonTrackedTarget bestTarget = calculateBestGamepeiceTarget(targetList);
  //   TargetInfo aprilTagInfo = m_aprilTagInfo[0];
  //   aprilTagInfo.yaw = bestTarget.getYaw();
  //   aprilTagInfo.pitch = bestTarget.getPitch();
  //   aprilTagInfo.timestamp = m_result.metadata.captureTimestampMicros;
  // }

  /**
   * The method to cache target data for AprilTags.
   *
   * @param targetList The list of targets that it pulls data from to cache.
   */
  // private void cacheForAprilTags(List<PhotonTrackedTarget> targetList) {
  //   for (PhotonTrackedTarget targetSeen : targetList) {
  //     int id = targetSeen.getFiducialId();
  //     TargetInfo aprilTagInfo = m_aprilTagInfo[id];
  //     aprilTagInfo.yaw = targetSeen.getYaw();
  //     aprilTagInfo.pitch = targetSeen.getPitch();
  //     aprilTagInfo.timestamp = m_result.metadata.captureTimestampMicros;
  //   }
  // }

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
      targetList.get(0).getPitch() + BEST_TARGET_PITCH_TOLERANCE.in(Degrees);
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
   * Returns whether or not the provided pose is within the field margin constants.
   * @param pose The pose that will be checked
   * @return Whether or not the provided pose is in the field
   */
  public static boolean isPoseInField(Pose3d pose) {
    return !(
      m_lastEstimatedPose.estimatedPose.getX() <
        -FIELD_BORDER_MARGIN.in(Meters) ||
      m_lastEstimatedPose.estimatedPose.getX() >
      FIELD_CONSTANTS.FIELD_LENGTH.in(Meters) +
      FIELD_BORDER_MARGIN.in(Meters) ||
      m_lastEstimatedPose.estimatedPose.getY() <
      -FIELD_BORDER_MARGIN.in(Meters) ||
      m_lastEstimatedPose.estimatedPose.getY() >
      FIELD_CONSTANTS.FIELD_LENGTH.in(Meters) +
      FIELD_BORDER_MARGIN.in(Meters) ||
      m_lastEstimatedPose.estimatedPose.getZ() < -Z_MARGIN.in(Meters) ||
      m_lastEstimatedPose.estimatedPose.getZ() > Z_MARGIN.in(Meters)
    );
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
  // public boolean isValidTarget(int targetId, long timeoutMs) {
  //   long now = System.currentTimeMillis();
  //   long then = now - timeoutMs;

  //   if (targetId >= m_aprilTagInfo.length) {
  //     return false;
  //   }
  //   TargetInfo target = m_aprilTagInfo[targetId];

  //   return (
  //     target.timestamp > then ||
  //     Math.abs(target.yaw) > MAX_ANGLE.in(Degrees) ||
  //     Math.abs(target.pitch) > MAX_ANGLE.in(Degrees)
  //   );
  // }

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
  // public int getBestTargetFiducialId() {
  //   return m_bestTargetFiducialId;
  // }

  /**
   * @param fiducialId The fiducial ID of the target to get the yaw of.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the desired targets yaw. <strong>Will be null if the cached data was invalid.
   */
  // public Angle getTargetYaw(int targetId, long timeoutMs) {
  //   if (isValidTarget(targetId, timeoutMs)) {
  //     return Units.Degrees.of(m_aprilTagInfo[targetId].yaw);
  //   }
  //   return null;
  // }

  /**
   * @param fiducialIds The list of fiducial IDs to check.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the yaw of the first valid target in the list, <strong>or null if none are valid.
   */
  // public Angle getTargetYaw(int[] targetIds, long timeoutMs) {
  //   for (int id : targetIds) {
  //     Angle yaw = getTargetYaw(id, timeoutMs);
  //     if (yaw != null) {
  //       return yaw;
  //     }
  //   }
  //   return null;
  // }

  /**
   * @param id The ID of the target to get the pitch of.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the desired targets pitch, <strong>will be null if the cached data was invalid.
   */
  // public Angle getTargetPitch(int targetId, long timeoutMs) {
  //   if (isValidTarget(targetId, timeoutMs)) {
  //     return Units.Degrees.of(m_aprilTagInfo[targetId].pitch);
  //   }
  //   return null;
  // }

  /**
   * @param fiducialIds The list of fiducial IDs to check.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the pitch of the first valid id in the list, <strong>or null if none are valid.
   */
  // public Angle getTargetPitch(int[] fiducialIds, long timeoutMs) {
  //   for (int id : fiducialIds) {
  //     Angle pitch = getTargetPitch(id, timeoutMs);
  //     if (pitch != null) {
  //       return pitch;
  //     }
  //   }
  //   return null;
  // }

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

    if (resultAmbiguity > MAX_AMBIGUITY_TOLERANCE) {
      return null;
    }
    if (resultReprojectionError > MAX_REPROJECTION_ERROR_PIXELS) {
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
