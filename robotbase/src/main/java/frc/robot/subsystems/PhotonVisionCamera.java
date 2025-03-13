package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.PHOTON_VISION.*;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Robot;
import frc.robot.commands.util.InitCamera;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Controls the photon vision camera options. */
public class PhotonVisionCamera {

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
    public PhotonVisionCamera camera = null;

    public TimestampedPNPInfo() {
      result = null;
      heading = null;
      headingTimestampSeconds = Double.NaN;
      camMatrix = null;
      distCoeefs = null;
      robotToCameraTransform = null;
      camera = null;
    }

    public void replaceInfo(
      PhotonPipelineResult result,
      Rotation3d heading,
      double headingTimestamp,
      Optional<Matrix<N3, N3>> camMatrix,
      Optional<Matrix<N8, N1>> distCoeefs,
      Transform3d robotToCamTransform,
      PhotonVisionCamera camera
    ) {
      this.result = result;
      this.heading = heading;
      this.headingTimestampSeconds = headingTimestamp;
      this.camMatrix = camMatrix;
      this.distCoeefs = distCoeefs;
      this.robotToCameraTransform = robotToCamTransform;
      this.camera = camera;
    }

    public void invalidateInfo() {
      result = null;
      heading = null;
      headingTimestampSeconds = Double.NaN;
      camMatrix = null;
      distCoeefs = null;
      robotToCameraTransform = null;
      camera = null;
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

  protected final record poseEstimate(
    Pose3d estimPose,
    double timestampSeconds,
    int targetsUsedNum,
    Matrix<N3, N1> stdDevs
  ) {
    public poseEstimate() {
      this(null, -1, -1, null);
    }

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
        !(this.estimPose == null) &&
        PhotonVisionCamera.isPoseInField(this.estimPose)
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

    public static poseEstimate averageOutValidEstimates(
      poseEstimate... estimates
    ) {
      if (estimates.length == 1) return estimates[0];
      Pose3d averagedPose = Pose3d.kZero;
      double averagedTimestamp = 0;
      int averageTargets = 0;
      int invalidEstimates = 0;
      Matrix<N3, N1> averageStdDevs = VecBuilder.fill(0, 0, 0);
      double averageCoordDev = 0;
      for (poseEstimate estimate : estimates) {
        if (estimate == null || !estimate.exists() || !estimate.isInField()) {
          invalidEstimates++;
          if (invalidEstimates >= estimates.length) return new poseEstimate();
          continue;
        }
        averagedPose.plus(new Transform3d(Pose3d.kZero, estimate.estimPose));
        averagedTimestamp += estimate.timestampSeconds;
        averageTargets += estimate.targetsUsedNum;
        averageCoordDev += estimate.stdDevs.get(0, 0);
      }
      averagedPose.div(estimates.length - invalidEstimates);
      averagedTimestamp /= estimates.length - invalidEstimates;
      averageTargets = Math.floorDiv(
        averageTargets,
        estimates.length - invalidEstimates
      );
      averageStdDevs.div(estimates.length - invalidEstimates);
      return new poseEstimate(
        averagedPose,
        averagedTimestamp,
        averageTargets,
        VecBuilder.fill(averageCoordDev, averageCoordDev, Double.MAX_VALUE)
      );
    }

    public static poseEstimate findConcensus(poseEstimate... estimates) {
      if (estimates.length == 1) {
        return estimates[0];
      }
      poseEstimate averageEstimate = averageOutValidEstimates(estimates);
      if (
        averageEstimate == null || !averageEstimate.exists()
      ) return new poseEstimate();
      Pose2d averagedPose = averageEstimate.estimPose.toPose2d();
      int invalidEstimates = 0;
      double averageDistBetweenEstimates = 0;
      for (poseEstimate estimate : estimates) {
        if (estimate == null || !estimate.exists() || !estimate.isInField()) {
          invalidEstimates++;
          continue;
        }
        if (invalidEstimates >= estimates.length) return new poseEstimate();
        Transform2d transform = new Transform2d(
          estimate.estimPose.toPose2d(),
          averagedPose
        );
        averageDistBetweenEstimates += Math.abs(
          transform.getTranslation().getNorm()
        );
      }
      averageDistBetweenEstimates /= estimates.length - invalidEstimates;
      if (averageDistBetweenEstimates > MAX_DIST_BETWEEN_ESTIMATES) {
        return new poseEstimate();
      }
      return averageEstimate;
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
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
      PRIMARY_STRATEGY,
      new Transform3d()
    );

  protected final Transform3d m_robotToCameraTranform;

  protected Optional<Matrix<N3, N3>> m_cameraMatrix;

  protected Optional<Matrix<N8, N1>> m_distCoeefs;

  /**
   * The latest pose estimation from all cameras
   */
  protected static EstimatedRobotPose m_lastEstimatedPose;

  private static PoseStrategy m_primaryStrategy = PRIMARY_STRATEGY;
  private static PoseStrategy m_fallbackStrategy = FALLBACK_STRATEGY;

  protected final NetworkTable m_poseOutputTable;
  protected final DoubleArrayPublisher m_poseFieldPub;
  protected final StringPublisher m_poseFieldTypePub;

  protected static final NetworkTable m_poseConcensusTable =
    NetworkTableInstance.getDefault().getTable("VisionPose-Combined");
  protected static final DoubleArrayPublisher m_poseConcensusFieldPub =
    m_poseConcensusTable.getDoubleArrayTopic("pose").publish();
  protected static final StringPublisher m_poseConcensusFieldTypePub =
    m_poseConcensusTable.getStringTopic(".type").publish();

  // m_poseFieldPub = m_poseOutputTable.getDoubleArrayTopic("pose").publish();
  //   m_poseFieldTypePub = m_poseOutputTable.getStringTopic(".type").publish();

  protected final NetworkTable m_photonTable;
  protected final DoubleArrayTopic m_intrinsicsTopic;
  protected final DoubleArrayTopic m_distortionTopic;

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
    Transform3d robotToCameraTransform
  ) {
    this(
      cameraName,
      robotToCameraTransform,
      Optional.ofNullable(null),
      Optional.ofNullable(null)
    );
    new InitCamera(this)
      .finallyDo(() -> {
        @SuppressWarnings("resource")
        Alert alert = new Alert(
          "CameraAlerts/" + m_camera.getName(),
          "Camera fully initialized",
          AlertType.kInfo
        );
        alert.set(true);
      })
      .schedule();
  }

  public PhotonVisionCamera(
    String cameraName,
    Transform3d robotToCameraTransform,
    Optional<Matrix<N3, N3>> cameraMatrix,
    Optional<Matrix<N8, N1>> distCoeefs
  ) {
    m_camera = new PhotonCamera(cameraName);
    m_robotCameras.add(this);

    // 1-22 correspond to apriltag fiducial IDs, 0 is for gamepeices.
    // m_aprilTagInfo = new TargetInfo[23];
    // for (int i = 0; i < m_aprilTagInfo.length; i++) {
    //   m_aprilTagInfo[i] = new TargetInfo();
    // }

    m_poseOutputTable = NetworkTableInstance.getDefault()
      .getTable("VisionPose-" + cameraName);
    m_poseFieldPub = m_poseOutputTable.getDoubleArrayTopic("pose").publish();
    m_poseFieldTypePub = m_poseOutputTable.getStringTopic(".type").publish();

    m_photonTable = NetworkTableInstance.getDefault()
      .getTable("photonvision")
      .getSubTable(cameraName);
    m_distortionTopic = m_photonTable.getDoubleArrayTopic("cameraDistortion");
    m_intrinsicsTopic = m_photonTable.getDoubleArrayTopic("cameraIntrinsics");

    m_robotToCameraTranform = robotToCameraTransform;
    m_cameraMatrix = cameraMatrix;
    m_distCoeefs = distCoeefs;
  }

  public boolean getDistCoeefs() {
    double[] distArray = m_distortionTopic.getEntry(null).get();
    if (distArray == null) {
      return false;
    }
    Matrix<N8, N1> distCoeefs = new Matrix<N8, N1>(
      Nat.N8(),
      Nat.N1(),
      distArray
    );

    m_distCoeefs = Optional.of(distCoeefs);
    return true;
  }

  public boolean getCameraMatrix() {
    double[] intrinsicsArray = m_intrinsicsTopic.getEntry(null).get();
    if (intrinsicsArray == null) {
      return false;
    }
    Matrix<N3, N3> cameraMatrix = new Matrix<N3, N3>(
      Nat.N3(),
      Nat.N3(),
      intrinsicsArray
    );
    m_cameraMatrix = Optional.of(cameraMatrix);
    return true;
  }

  /**
   * Updates all instances of {@link PhotonVisionCamera} with the latest result, and updates the pose with the updated pnpInfo.
   * <p><h1> For the top of {@link Robot#robotPeriodic() the robot periodic} only.
   */
  public static void updateAllCameras() {
    poseEstimate[] estimates = new poseEstimate[PNP_INFO_STORAGE_AMOUNT];
    boolean updatePose = SmartDashboard.getBoolean(
      "Toggle Pose Estimation",
      false
    );
    for (PhotonVisionCamera camera : m_robotCameras) {
      if (camera.isConnected()) {
        camera.updateResult();
      } else if (
        RobotModeTriggers.disabled().getAsBoolean() &&
        RobotController.getMeasureTime().in(Seconds) % 5 <= 0.1
      ) {
        System.err.println(
          "CAMERA " +
          camera.m_camera.getName() +
          " IS DISCONNECTED! ***** TELL MAX! *****"
        );
      }
    }
    for (int i = 0; i < m_pnpInfo.length; i++) {
      if (m_pnpInfo[i].result == null) {
        continue;
      }
      estimates[i] = estimatePoseWithPNPInfo(m_pnpInfo[i]);
      m_pnpInfo[i].invalidateInfo();
    }
    updatePoseFromPoseEstimates(updatePose, estimates);
  }

  /**
   * Sets the primary pose strategy for all cameras.<p> <h1> Do not call this unless you know what your doing!</h1>
   * @param strategy The strategy that will become the primary strategy
   */
  public static void setPrimaryStrategy(PoseStrategy strategy) {
    m_primaryStrategy = strategy;
    m_poseEstimator.setPrimaryStrategy(strategy);
  }

  /**
   * Sets the fallback pose strategy for all cameras.<p> <h1> Do not call this unless you know what your doing!</h1>
   * @param strategy The strategy that will become the fallback strategy
   */
  public static void setFallbackStrategy(PoseStrategy strategy) {
    m_fallbackStrategy = strategy;
    m_poseEstimator.setMultiTagFallbackStrategy(strategy);
  }

  /**
   * Fetches the latest pipeline result for this instance
   */
  private void updateResult() {
    if (!m_camera.isConnected()) {
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
    switch (m_primaryStrategy) {
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
    switch (m_fallbackStrategy) {
      case CONSTRAINED_SOLVEPNP, PNP_DISTANCE_TRIG_SOLVE:
        // if our primary strategy already needs us to update the heading, we dont want to call more hardware.
        // they get upset if you call them enough, and its a waste of time.
        if (
          m_primaryStrategy == PoseStrategy.CONSTRAINED_SOLVEPNP ||
          m_primaryStrategy == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
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

  private static poseEstimate estimatePoseWithPNPInfo(
    TimestampedPNPInfo pnpInfo
  ) {
    preparePoseEstimator(
      pnpInfo.robotToCameraTransform,
      pnpInfo.heading,
      pnpInfo.headingTimestampSeconds
    );
    EstimatedRobotPose estimatedPose = m_poseEstimator
      .update(
        pnpInfo.result,
        pnpInfo.camMatrix,
        pnpInfo.distCoeefs,
        POSE_EST_PARAMS
      )
      .orElse(null);
    if (estimatedPose == null) {
      return new poseEstimate();
    }
    publishPose(estimatedPose.estimatedPose.toPose2d(), pnpInfo.camera);
    double averageTargetDistance = 0;
    for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
      averageTargetDistance += target.getBestCameraToTarget().getX();
    }
    averageTargetDistance /= estimatedPose.targetsUsed.size();
    if (!isPoseInField(estimatedPose.estimatedPose)) return new poseEstimate();
    // if (estimatedPose.targetsUsed.size() < 2) return new poseEstimate();

    // the higher the confidence is, the less the estimated measurment is trusted.
    double velocityConf =
      MAGIC_VEL_CONF_ADDEND +
      Math.abs(
        Robot.swerve.getAbsoluteTranslationalVelocity().in(MetersPerSecond)
      );

    double coordinateConfidence = Math.pow(
      estimatedPose.targetsUsed.size() *
      ((averageTargetDistance / 2) * velocityConf),
      MAGIC_VEL_CONF_EXPONENT
    );
    Pose2d estimatedPose2d = estimatedPose.estimatedPose.toPose2d();
    // if estimated pose is too far from current pose
    if (
      new Transform2d(Robot.swerve.getFieldRelativePose2d(), estimatedPose2d)
        .getTranslation()
        .getNorm() >
      MAX_DISTANCE_FROM_CURR_POSE_METERS
    ) {
      // and were not disabled, throw it out
      if (!RobotModeTriggers.disabled().getAsBoolean()) {
        return new poseEstimate();
      }
    }
    m_lastEstimatedPose = estimatedPose;
    return new poseEstimate(
      estimatedPose,
      VecBuilder.fill(
        coordinateConfidence * X_STD_DEV_COEFFIECIENT,
        coordinateConfidence * Y_STD_DEV_COEFFIECIENT,
        Double.MAX_VALUE // Theta conf, should never change the gyro heading
      )
    );
  }

  private static void updatePoseFromPoseEstimates(
    boolean updatePose,
    poseEstimate... estimates
  ) {
    poseEstimate averageEstimate = estimates.length > 1
      ? poseEstimate.findConcensus(estimates)
      : estimates[0];
    if (averageEstimate == null || !averageEstimate.exists()) return;
    m_poseConcensusFieldTypePub.set("Field2d");
    Pose2d pose = averageEstimate.estimPose.toPose2d();
    m_poseConcensusFieldPub.accept(
      new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() }
    );
    if (!updatePose) return;

    if (!averageEstimate.isInField()) return;
    Robot.swerve.addVisionMeasurement(
      averageEstimate.estimPose.toPose2d(),
      averageEstimate.timestampSeconds,
      averageEstimate.stdDevs
    );
  }

  public static void publishPose(Pose2d pose, PhotonVisionCamera camera) {
    if (pose == null || camera == null) {
      return;
    }
    camera.m_poseFieldTypePub.set("Field2d");
    camera.m_poseFieldPub.accept(
      new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() }
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
    if (result == null || !result.hasTargets()) {
      return;
    }
    double frameTimeSeconds = Utils.fpgaToCurrentTime(
      result.getTimestampSeconds()
    );
    double currTimeSeconds = Utils.getCurrentTimeSeconds();
    // if result is older than allowed, do not store it
    if (frameTimeSeconds <= currTimeSeconds - PNP_INFO_VALID_TIME.in(Seconds)) {
      return;
    }
    if (
      Math.abs(Robot.swerve.getAngularVelocity().in(RadiansPerSecond)) >
      MAX_ACCEPTABLE_ROTATIONAL_VELOCITY.in(RadiansPerSecond)
    ) return;
    if (
      Math.abs(
        Robot.swerve.getAbsoluteTranslationalVelocity().in(MetersPerSecond)
      ) >
      MAX_ACCEPTABLE_TRANSLATIONAL_VELOCITY.in(MetersPerSecond)
    ) return;
    Rotation3d heading = new Rotation3d(
      Robot.swerve.getFieldRelativePose2d().getRotation()
    );
    double headingTimestampSeconds = Utils.fpgaToCurrentTime(
      result.getTimestampSeconds()
    );
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
            m_robotToCameraTranform,
            this
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
    if (indexToReplace != -1) {
      m_pnpInfo[indexToReplace].replaceInfo(
          result,
          heading,
          headingTimestampSeconds,
          m_cameraMatrix,
          m_distCoeefs,
          m_robotToCameraTranform,
          this
        );
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
      pose.getX() < -FIELD_BORDER_MARGIN.in(Meters) ||
      pose.getX() >
      FIELD_CONSTANTS.FIELD_LENGTH.plus(FIELD_BORDER_MARGIN).in(Meters) ||
      pose.getY() < -FIELD_BORDER_MARGIN.in(Meters) ||
      pose.getY() >
      FIELD_CONSTANTS.FIELD_WIDTH.plus(FIELD_BORDER_MARGIN).in(Meters)
    );
  }

  // public static Matrix<N8, N1> getDistCoeefsFromNT(String cameraName) {
  //   NetworkTable table = NetworkTableInstance.getDefault()
  //     .getTable("photonvision");
  //   DoubleArrayTopic topic = table.getDoubleArrayTopic(
  //     cameraName + "/cameraDistortion"
  //   );
  //   double[] distortionFromNT = topic
  //     .genericSubscribe()
  //     .getDoubleArray(new double[] {Double.NaN});
  //   if(Double.isNaN(distortionFromNT[0]));
  // }

  /**
   * @return Whether or not the camera is connected.
   */
  public boolean isConnected() {
    return m_camera.isConnected();
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
   * Gets the last estimated pose from PV's pose estimator.
   *
   * <p>Should only be used if the camera does not see more than 1 april tag, if it does, use
   * getPNPResult instead, as it is more accurate.
   *
   * @return The robots estimated pose, if it has any april tag targets. <strong>Can be null at various points.</strong>
   */
  public static EstimatedRobotPose getLastEstimatedPose() {
    return m_lastEstimatedPose;
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
