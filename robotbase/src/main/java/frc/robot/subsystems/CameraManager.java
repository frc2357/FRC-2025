package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.PHOTON_VISION.*;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.subsystems.PhotonVisionCamera.TimestampedPNPInfo;
import frc.robot.util.CollisionDetection;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraManager {

  public final record poseEstimate(
    Pose3d estimPose,
    double timestampSeconds,
    int targetsUsedNum,
    Matrix<N3, N1> stdDevs
  ) {
    public poseEstimate(EstimatedRobotPose estim, Matrix<N3, N1> stdDevs) {
      this(
        estim.estimatedPose,
        estim.timestampSeconds,
        estim.targetsUsed.size(),
        stdDevs
      );
    }

    public boolean isInField() {
      return (
        this.estimPose != null &&
        CollisionDetection.isPoseInField(this.estimPose)
      );
    }

    public boolean exists() {
      return (
        this.estimPose != null &&
        this.stdDevs != null &&
        this.timestampSeconds > 0 &&
        this.targetsUsedNum > 0
      );
    }

    /**
     * Takes an arbitrary number of {@link poseEstimate} objects and finds the average between all characteristics of the estimates.
     * @param estimates The estimates to average out. Must contain valid estimates only, and not be null.
     * @return The averaged out estimate
     */
    public static poseEstimate averageOutEstimates(poseEstimate... estimates) {
      if (estimates.length == 1) return estimates[0];
      Translation3d aveTranslation = new Translation3d();
      double averagedTimestamp = 0;
      int cummulativeTargets = 0;
      double averageCoordDev = 0;
      for (poseEstimate estimate : estimates) {
        aveTranslation = aveTranslation.plus(
          estimate.estimPose.getTranslation()
        );
        averagedTimestamp += estimate.timestampSeconds;
        cummulativeTargets += estimate.targetsUsedNum;
        averageCoordDev += estimate.stdDevs.get(0, 0);
      }
      aveTranslation = aveTranslation.div(estimates.length);
      averagedTimestamp /= estimates.length;
      averageCoordDev /= estimates.length;
      return new poseEstimate(
        new Pose3d(aveTranslation, estimates[0].estimPose.getRotation()),
        averagedTimestamp,
        cummulativeTargets,
        VecBuilder.fill(averageCoordDev, averageCoordDev, 100000)
      );
    }

    /** Takes an arbitrary number of {@link poseEstimate poseEstimates} and finds the most commonly agreed upon pose.
     * @param estimates The estimates to draw from, can contain invalid estimates.
     * @return A pose estimate of the average estimate, as long as all the estimates are close enough together
     */
    public static poseEstimate findConcensus(poseEstimate... estimates) {
      if (estimates == null) return null;
      if (estimates.length == 1) return estimates[0];
      poseEstimate averageEstimate = averageOutEstimates(estimates);
      if (averageEstimate == null || !averageEstimate.exists()) return null;
      double averageDistBetweenEstimates = 0;
      for (poseEstimate estimate : estimates) {
        averageDistBetweenEstimates += estimate.estimPose
          .getTranslation()
          .getDistance(averageEstimate.estimPose.getTranslation());
      }
      averageDistBetweenEstimates /= estimates.length;
      if (
        averageDistBetweenEstimates >= MAX_DIST_BETWEEN_ESTIMATES.in(Meters)
      ) return null;
      return averageEstimate;
    }

    public static poseEstimate[] findValidEstimates(
      Rotation2d desiredHeading,
      poseEstimate... estimates
    ) {
      ArrayList<poseEstimate> validEstimates = new ArrayList<poseEstimate>();
      for (poseEstimate estimate : estimates) {
        if (estimate == null || !estimate.isValid()) {
          continue;
        }
        validEstimates.add(estimate);
      }
      if (validEstimates.size() == 0) {
        return null;
      }
      return validEstimates.toArray(new poseEstimate[validEstimates.size()]);
    }

    public boolean isValid() {
      return (
        this.exists() &&
        this.isInField() &&
        (this.timestampSeconds + ESTIMATE_TIMEOUT.in(Seconds) >
          RobotController.getMeasureFPGATime().in(Seconds))
      );
    }
  }

  public class TargetInfo {

    public double yaw, pitch, skew;
    public Transform3d camToTargetTransform;
    public long timestamp;
    public Pose2d targetFieldRelativePose;
    public PhotonVisionCamera camera;

    public TargetInfo(
      double yaw,
      double pitch,
      double skew,
      Transform3d camToTargetTransform,
      long timestamp,
      PhotonVisionCamera newCamera
    ) {
      this.yaw = yaw;
      this.pitch = pitch;
      this.skew = skew;
      this.camToTargetTransform = camToTargetTransform;
      this.timestamp = timestamp;
      if (newCamera == null) {
        this.targetFieldRelativePose = null;
        return;
      }
      this.targetFieldRelativePose = new Pose3d(
        Robot.swerve.getFieldRelativePose2d()
      )
        .plus(newCamera.m_robotToCameraTranform) //fieldToRobot -> fieldToCamera
        .plus(camToTargetTransform) //fieldToCamera -> fieldToTarget
        .toPose2d();
      this.camera = newCamera;
    }

    public boolean isValid(long timeoutMs) {
      long now = RobotController.getFPGATime();
      long then = now - timeoutMs;

      return (
        camera != null &&
        camToTargetTransform != null &&
        targetFieldRelativePose != null &&
        timestamp > then &&
        (!Double.isNaN(yaw) && Math.abs(yaw) > MAX_ANGLE.in(Degrees)) &&
        (!Double.isNaN(pitch) && Math.abs(pitch) > MAX_ANGLE.in(Degrees)) &&
        !Double.isNaN(skew)
      );
    }

    public void replace(PhotonTrackedTarget target, PhotonVisionCamera camera) {
      this.yaw = target.yaw;
      this.pitch = target.pitch;
      this.skew = target.skew;
      this.camera = camera;
      this.camToTargetTransform = target.bestCameraToTarget;
      this.timestamp = RobotController.getFPGATime();
      this.targetFieldRelativePose = new Pose3d(
        Robot.swerve.getFieldRelativePose2d()
      )
        .plus(camera.m_robotToCameraTranform) //camToTarget -> robotToTarget
        .plus(camToTargetTransform) //robotToTarget -> fieldOriginToTarget
        .toPose2d();
    }
  }

  private final Map<PhotonVisionCamera, poseEstimate> m_robotCameras =
    new LinkedHashMap<PhotonVisionCamera, poseEstimate>();

  private final TargetInfo[] m_aprilTagInfo = new TargetInfo[23];

  private final NetworkTable m_poseConcensusTable =
    NetworkTableInstance.getDefault().getTable("VisionPose-Combined");
  private final DoubleArrayPublisher m_poseConcensusFieldPub =
    m_poseConcensusTable.getDoubleArrayTopic("pose").publish();
  private final StringPublisher m_poseConcensusFieldTypePub =
    m_poseConcensusTable.getStringTopic(".type").publish();

  private final String m_alertGroupName = "Camera Manager";
  private final Alert m_alertEstimateInfo = new Alert(
    m_alertGroupName,
    "Initializing",
    AlertType.kInfo
  );
  private final Alert m_alertTargetNumInfo = new Alert(
    m_alertGroupName,
    "Initializing",
    AlertType.kInfo
  );

  private Pose3d m_lastEstimatedPose;

  private PoseStrategy m_primaryStrat = PRIMARY_STRATEGY;
  private PoseStrategy m_fallbackStrat = FALLBACK_STRATEGY;

  public CameraManager() {
    for (int i = 0; i < m_aprilTagInfo.length; i++) {
      m_aprilTagInfo[i] = new TargetInfo(
        Double.NaN,
        Double.NaN,
        Double.NaN,
        Transform3d.kZero,
        -1,
        null
      );
    }
    m_alertEstimateInfo.set(false);
    m_alertTargetNumInfo.set(true);
    m_lastEstimatedPose = Pose3d.kZero;
  }

  /**
   * Updates all instances of {@link PhotonVisionCamera} with the latest result, and updates the pose with the updated pnpInfo.
   */
  public void updateAllCameras() {
    boolean updatePose = SmartDashboard.getBoolean(
      "Toggle Pose Estimation",
      false
    );
    for (var entry : m_robotCameras.entrySet()) {
      var camera = entry.getKey();
      if (camera.isConnected()) camera.updateResult();
      else if (
        RobotModeTriggers.disabled().getAsBoolean() &&
        RobotController.getMeasureTime().in(Seconds) % 5 <= 0.1
      ) {
        System.err.println(
          "CAMERA " +
          camera.m_camera.getName() +
          " IS DISCONNECTED! ***** TELL MAX! *****"
        );
        continue;
      }
    }
    //checks if we have estimates from all cameras before continuing.
    for (var entry : m_robotCameras.entrySet()) {
      // if were lacking an estimate from a connected camera, skip updating
      if (
        entry.getKey().isConnected() &&
        (entry.getValue() == null || !entry.getValue().isValid())
      ) return;
    }

    updatePoseFromPoseEstimates(
      updatePose,
      m_robotCameras.values().toArray(new poseEstimate[m_robotCameras.size()])
    );
  }

  public PhotonVisionCamera createCamera(
    String name,
    Transform3d robotToCameraTransform
  ) {
    PhotonVisionCamera cam = new PhotonVisionCamera(
      name,
      robotToCameraTransform,
      this::processResult
    );
    m_robotCameras.put(cam, null);
    cam.m_poseEstimator.setPrimaryStrategy(m_primaryStrat);
    cam.m_poseEstimator.setMultiTagFallbackStrategy(m_fallbackStrat);
    return cam;
  }

  private void preparePoseEstimator(TimestampedPNPInfo infoToPrepFor) {
    PhotonPoseEstimator estimator = infoToPrepFor.camera().m_poseEstimator;
    // change pose estimator settings to be correct for the provided info
    estimator.setRobotToCameraTransform(infoToPrepFor.robotToCameraTransform());
    estimator.setReferencePose(Robot.swerve.getFieldRelativePose2d());
    estimator.addHeadingData(
      infoToPrepFor.headingTimestampSeconds(),
      infoToPrepFor.heading()
    );
  }

  private poseEstimate estimatePoseWithPNPInfo(TimestampedPNPInfo pnpInfo) {
    if (pnpInfo == null || !pnpInfo.exists()) return null;
    preparePoseEstimator(pnpInfo);
    EstimatedRobotPose estimate = pnpInfo
      .camera()
      .m_poseEstimator.update(pnpInfo.result())
      .orElse(null);

    if (estimate == null) return null;

    pnpInfo.camera().publishPose(estimate.estimatedPose.toPose2d());

    // if estimate is out of the field, throw it away
    if (!CollisionDetection.isPoseInField(estimate.estimatedPose)) return null;

    double averageTargetDistance = 0;

    for (PhotonTrackedTarget target : estimate.targetsUsed) {
      averageTargetDistance += Math.abs(
        target.getBestCameraToTarget().getTranslation().getNorm()
      );
    }
    averageTargetDistance /= estimate.targetsUsed.size();
    // the higher the confidence is, the less the estimated measurment is trusted.
    double velocityConf =
      MAGIC_VEL_CONF_ADDEND +
      Math.abs(
        Robot.swerve.getAbsoluteTranslationalVelocity().in(MetersPerSecond)
      );

    double coordinateConfidence = Math.pow(
      estimate.targetsUsed.size() *
      ((averageTargetDistance / 2) * velocityConf),
      MAGIC_VEL_CONF_EXPONENT
    );

    return new poseEstimate(
      estimate,
      VecBuilder.fill(
        coordinateConfidence * X_STD_DEV_COEFFIECIENT,
        coordinateConfidence * Y_STD_DEV_COEFFIECIENT,
        Double.MAX_VALUE // Theta conf, should never change the gyro heading
      )
    );
  }

  private void updatePoseFromPoseEstimates(
    boolean updatePose,
    poseEstimate... estimates
  ) {
    estimates = poseEstimate.findValidEstimates(
      Robot.swerve.getFieldRelativePose2d().getRotation(),
      estimates
    );
    if (estimates == null) {
      m_alertEstimateInfo.setText("No Valid Estimates");
      m_alertEstimateInfo.set(true);
      return;
    }
    m_alertEstimateInfo.set(false);
    poseEstimate averageEstimate = estimates.length > 1
      ? poseEstimate.findConcensus(estimates)
      : estimates[0];
    if (averageEstimate == null || !averageEstimate.exists()) return;

    m_poseConcensusFieldTypePub.set("Field2d");

    Pose2d pose = averageEstimate.estimPose.toPose2d();
    m_alertTargetNumInfo.setText(
      "Target Num: " + averageEstimate.targetsUsedNum
    );
    m_poseConcensusFieldPub.accept(
      new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() }
    );
    // if estimate isnt in the field, throw it away
    if (!averageEstimate.isInField()) return;
    m_lastEstimatedPose = averageEstimate.estimPose;
    // if we dont want to update the pose, throw it away
    if (!updatePose) return;
    double measuredTime = Utils.fpgaToCurrentTime(
      averageEstimate.timestampSeconds
    );
    // if estimated pose is too far from swerve pose
    if (
      Math.abs(
        Robot.swerve
          .getFieldRelativePose2d()
          .getTranslation()
          .getDistance(averageEstimate.estimPose.toPose2d().getTranslation())
      ) >
      MAX_DIST_FROM_CURR_POSE.in(Meters)
    ) {
      // and is NOT disabled, throw away pose.
      if (!RobotModeTriggers.disabled().getAsBoolean()) {
        return;
      }
    }
    Robot.swerve.addVisionMeasurement(
      averageEstimate.estimPose.toPose2d(),
      measuredTime,
      averageEstimate.stdDevs
    );
  }

  public void setPrimaryStrategy(PoseStrategy newStrategy) {
    m_primaryStrat = newStrategy;
    m_robotCameras
      .keySet()
      .forEach((var cam) -> cam.m_poseEstimator.setPrimaryStrategy(newStrategy)
      );
  }

  public void setFallbackStrategy(PoseStrategy newStrategy) {
    m_fallbackStrat = newStrategy;
    m_robotCameras
      .keySet()
      .forEach((var cam) ->
        cam.m_poseEstimator.setMultiTagFallbackStrategy(newStrategy)
      );
  }

  public Pose3d getLastEstimatedPose() {
    return m_lastEstimatedPose;
  }

  private void cacheForGamepeices(
    List<PhotonTrackedTarget> targetList,
    PhotonVisionCamera camera
  ) {
    PhotonTrackedTarget bestTarget = calculateBestGamepeiceTarget(targetList);
    m_aprilTagInfo[0].replace(bestTarget, camera);
  }

  private void cacheForAprilTags(
    List<PhotonTrackedTarget> targets,
    PhotonVisionCamera camera
  ) {
    for (PhotonTrackedTarget target : targets) {
      m_aprilTagInfo[target.fiducialId].replace(target, camera);
    }
  }

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
    }
    return bestTarget;
  }

  private void processResult(
    PhotonVisionCamera camera,
    PhotonPipelineResult result
  ) {
    var info = createPNPInfo(result, camera);
    m_robotCameras.put(camera, estimatePoseWithPNPInfo(info));
    // storePNPInfo(info);
    if (result.getBestTarget().objDetectId != -1) {
      cacheForGamepeices(result.targets, camera);
    } else {
      cacheForAprilTags(result.targets, camera);
    }
  }

  private TimestampedPNPInfo createPNPInfo(
    PhotonPipelineResult result,
    PhotonVisionCamera camera
  ) {
    // if the provided result has no targets, it has no value, so we do not want to store it
    if (result == null || !result.hasTargets()) {
      return null;
    }
    // if rotating too fast, dont create info
    if (
      Math.abs(Robot.swerve.getAngularVelocity().in(RadiansPerSecond)) >
      MAX_ACCEPTABLE_ROTATIONAL_VELOCITY.in(RadiansPerSecond)
    ) return null;
    // if translating too fast, dont create info
    if (
      Robot.swerve.getAbsoluteTranslationalVelocity().in(MetersPerSecond) >
      MAX_ACCEPTABLE_TRANSLATIONAL_VELOCITY.in(MetersPerSecond)
    ) return null;

    Rotation3d heading = new Rotation3d(
      Robot.swerve.getFieldRelativePose2d().getRotation()
    );
    double headingTimestampSeconds = Utils.fpgaToCurrentTime(
      result.getTimestampSeconds()
    );
    return new TimestampedPNPInfo(
      result,
      heading,
      headingTimestampSeconds,
      camera.m_robotToCameraTranform,
      camera
    );
  }

  /**
   * Returns a field relative pose of a target based on where the robot thinks it is, and the provided camera transforms
   * @param targetId The targets fiducial ID
   * @param timeoutMs The amount of milliseconds past which the target information is deemed expired
   * @return Returns the desired targets field relative pose, <strong> will return null if cached data was invalid. </strong>
   */
  public Pose2d getFieldRelativeTargetPose(int targetId, long timeoutMs) {
    return isValidTarget(targetId, timeoutMs)
      ? m_aprilTagInfo[targetId].targetFieldRelativePose
      : null;
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
    return m_aprilTagInfo[targetId].isValid(timeoutMs);
  }

  /**
   * @param fiducialId The fiducial ID of the target to get the yaw of.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the desired targets yaw. <strong>Will be null if the cached data was invalid.
   */
  public Angle getTargetYaw(int targetId, long timeoutMs) {
    return isValidTarget(targetId, timeoutMs)
      ? Units.Degrees.of(m_aprilTagInfo[targetId].yaw)
      : null;
  }

  /**
   * @param id The ID of the target to get the pitch of.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the desired targets pitch, <strong>will return null if the cached data was invalid.</strong>
   */
  public Angle getTargetPitch(int targetId, long timeoutMs) {
    return isValidTarget(targetId, timeoutMs)
      ? Units.Degrees.of(m_aprilTagInfo[targetId].pitch)
      : null;
  }
}
