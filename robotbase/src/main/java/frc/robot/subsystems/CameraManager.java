package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.FIELD.REEF.BLUE_REEF_TAGS;
import static frc.robot.Constants.FIELD.REEF.RED_REEF_TAGS;
import static frc.robot.Constants.FIELD.REEF.REEF_BRANCHES;
import static frc.robot.Constants.PHOTON_VISION.*;

import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.DRIVE_TO_POSE.BRANCH_GOAL;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Constants.PHOTON_VISION;
import frc.robot.Constants.ROBOT_CONFIGURATION;
import frc.robot.Robot;
import frc.robot.subsystems.PhotonVisionCamera.TimestampedPNPInfo;
import frc.robot.util.CollisionDetection;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
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
      Pose3d averagedPose = Pose3d.kZero;
      double averagedTimestamp = 0;
      int averageTargets = 0;
      Matrix<N3, N1> averageStdDevs = VecBuilder.fill(0, 0, 0);
      double averageCoordDev = 0;
      for (poseEstimate estimate : estimates) {
        averagedPose = averagedPose.plus(
          new Transform3d(Pose3d.kZero, estimate.estimPose)
        );
        averagedTimestamp += estimate.timestampSeconds;
        averageTargets += estimate.targetsUsedNum;
        averageCoordDev += estimate.stdDevs.get(0, 0);
      }
      averagedPose.div(estimates.length);
      averagedTimestamp /= estimates.length;
      averageTargets = Math.floorDiv(averageTargets, estimates.length);
      averageStdDevs.div(estimates.length);
      return new poseEstimate(
        averagedPose,
        averagedTimestamp,
        averageTargets,
        VecBuilder.fill(averageCoordDev, averageCoordDev, Double.MAX_VALUE)
      );
    }

    /** Takes an arbitrary number of {@link poseEstimate poseEstimates} and finds the most commonly agreed upon pose.
     * @param estimates The estimates to draw from, can contain invalid estimates.
     * @return A pose estimate of the average estimate, as long as all the estimates are close enough together
     */
    public static poseEstimate findConcensus(poseEstimate... estimates) {
      estimates = findValidEstimates(estimates);
      if (estimates == null) return null;
      if (estimates.length == 1) return estimates[0];
      poseEstimate averageEstimate = averageOutEstimates(estimates);
      if (averageEstimate == null || !averageEstimate.exists()) return null;
      Pose2d averagedPose = averageEstimate.estimPose.toPose2d();
      double averageDistBetweenEstimates = 0;
      for (poseEstimate estimate : estimates) {
        Transform2d transform = new Transform2d(
          estimate.estimPose.toPose2d(),
          averagedPose
        );
        averageDistBetweenEstimates += Math.abs(
          transform.getTranslation().getNorm()
        );
      }
      averageDistBetweenEstimates /= estimates.length;
      if (
        averageDistBetweenEstimates >
        PHOTON_VISION.MAX_DIST_BETWEEN_ESTIMATES.in(Meters)
      ) return null;
      return averageEstimate;
    }

    public static poseEstimate[] findValidEstimates(poseEstimate... estimates) {
      ArrayList<poseEstimate> validEstimates = new ArrayList<poseEstimate>();
      for (poseEstimate estimate : estimates) {
        if (estimate != null && estimate.isValid()) {
          validEstimates.add(estimate);
        }
      }
      if (validEstimates.size() == 0) {
        return null;
      }
      return validEstimates.toArray(new poseEstimate[validEstimates.size()]);
    }

    public boolean isValid() {
      return this.exists() && this.isInField();
    }
  }

  private class TargetInfo {

    public double yaw, pitch, skew;
    public Transform3d camToTargetTransform;
    public long timestamp;
    public Pose3d targetFieldRelativePose;
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
        .plus(newCamera.m_robotToCameraTranform) //camToTarget -> robotToTarget
        .plus(camToTargetTransform); //robotToTarget -> fieldOriginToTarget
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
        .plus(camToTargetTransform); //robotToTarget -> fieldOriginToTarget
    }
  }

  private final ArrayList<PhotonVisionCamera> m_robotCameras = new ArrayList<
    PhotonVisionCamera
  >();

  private final PhotonPoseEstimator m_poseEstimator = new PhotonPoseEstimator(
    FIELD_CONSTANTS.APRIL_TAG_LAYOUT,
    PRIMARY_STRATEGY,
    new Transform3d()
  );

  private final TimestampedPNPInfo[] m_pnpInfo =
    new TimestampedPNPInfo[PNP_INFO_STORAGE_AMOUNT];

  private final TargetInfo[] m_aprilTagInfo = new TargetInfo[23];

  /**
   * Holds all the poses for all the branches and options in the BRANCH enum
   */
  private final Pose2d[] m_branchPositions = new Pose2d[13];

  private final NetworkTable m_poseConcensusTable =
    NetworkTableInstance.getDefault().getTable("VisionPose-Combined");
  private final DoubleArrayPublisher m_poseConcensusFieldPub =
    m_poseConcensusTable.getDoubleArrayTopic("pose").publish();
  private final StringPublisher m_poseConcensusFieldTypePub =
    m_poseConcensusTable.getStringTopic(".type").publish();

  private MutTime m_lastPoseUpdateTime;
  private EstimatedRobotPose m_lastEstimatedPose;

  private PoseStrategy m_primaryStrat = PRIMARY_STRATEGY;
  private PoseStrategy m_fallbackStrat = FALLBACK_STRATEGY;

  public CameraManager() {
    m_lastPoseUpdateTime = new MutTime(0, 0, Seconds);
    m_poseEstimator.setPrimaryStrategy(m_primaryStrat);
    m_poseEstimator.setMultiTagFallbackStrategy(m_fallbackStrat);
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
    Arrays.fill(m_branchPositions, Pose2d.kZero);
  }

  /**
   * Updates all instances of {@link PhotonVisionCamera} with the latest result, and updates the pose with the updated pnpInfo.
   */
  public void updateAllCameras() {
    poseEstimate[] estimates = new poseEstimate[PNP_INFO_STORAGE_AMOUNT];
    boolean updatePose = SmartDashboard.getBoolean(
      "Toggle Pose Estimation",
      false
    );
    for (PhotonVisionCamera camera : m_robotCameras) {
      if (camera.isConnected()) camera.updateResult();
      else if (
        RobotModeTriggers.disabled().getAsBoolean() &&
        RobotController.getMeasureTime().in(Seconds) % 5 <= 0.1
      ) System.err.println(
        "CAMERA " +
        camera.m_camera.getName() +
        " IS DISCONNECTED! ***** TELL MAX! *****"
      );
    }
    for (int i = 0; i < m_pnpInfo.length; i++) {
      if (m_pnpInfo[i] == null) continue;

      estimates[i] = estimatePoseWithPNPInfo(m_pnpInfo[i]);
      m_pnpInfo[i] = null;
    }
    updatePoseFromPoseEstimates(updatePose, estimates);
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
    m_robotCameras.add(cam);
    return cam;
  }

  private void preparePoseEstimator(TimestampedPNPInfo infoToPrepFor) {
    m_poseEstimator.addHeadingData(
      infoToPrepFor.headingTimestampSeconds(),
      infoToPrepFor.heading()
    );

    // change pose estimator settings to be correct for the provided info
    m_poseEstimator.setRobotToCameraTransform(
      infoToPrepFor.robotToCameraTransform()
    );
    m_poseEstimator.setReferencePose(Robot.swerve.getFieldRelativePose2d());
  }

  private poseEstimate estimatePoseWithPNPInfo(TimestampedPNPInfo pnpInfo) {
    if (pnpInfo == null || !pnpInfo.exists()) return null;
    preparePoseEstimator(pnpInfo);
    EstimatedRobotPose estimate = m_poseEstimator
      .update(pnpInfo.result())
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

    // if estimated pose is too far from current pose
    m_lastEstimatedPose = estimate;
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
    // get the most common estimate
    poseEstimate averageEstimate = estimates.length > 1
      ? poseEstimate.findConcensus(estimates)
      : estimates[0];

    if (averageEstimate == null || !averageEstimate.exists()) return;

    m_poseConcensusFieldTypePub.set("Field2d");
    Pose2d pose = averageEstimate.estimPose.toPose2d();
    m_poseConcensusFieldPub.accept(
      new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() }
    );
    // if we dont want to update the pose, throw it away
    if (!updatePose) return;
    // if estimate isnt in the field, throw it away
    if (!averageEstimate.isInField()) return;
    double measuredTime = Utils.fpgaToCurrentTime(
      averageEstimate.timestampSeconds
    );
    // if estimated pose it too far from swerve pose
    if (
      new Transform2d(
        Robot.swerve.getFieldRelativePose2d(),
        averageEstimate.estimPose.toPose2d()
      )
        .getTranslation()
        .getNorm() >
      MAX_DIST_FROM_CURR_POSE.in(Meters)
    ) {
      if (RobotModeTriggers.disabled().getAsBoolean()) {
        Robot.swerve.resetPose(pose);
        return;
      }
      // and we have updated the pose recently, and were not disabled, throw it out
      if (
        measuredTime >
        m_lastPoseUpdateTime.plus(UPDATE_POSE_INTERVALS).in(Seconds)
      ) return;
    }

    Robot.swerve.addVisionMeasurement(
      averageEstimate.estimPose.toPose2d(),
      measuredTime,
      averageEstimate.stdDevs
    );
    m_lastPoseUpdateTime.mut_replace(measuredTime, Seconds);
  }

  private void storePNPInfo(TimestampedPNPInfo pnpInfo) {
    if (pnpInfo == null || !pnpInfo.exists()) return;
    int indexToReplace = -1;
    if (pnpInfo.result().targets.size() < MIN_ALLOWED_TARGETS) return;
    for (int i = 0; i < m_pnpInfo.length; i++) {
      // if selected info does not exist, replace it and stop the loop
      if (m_pnpInfo[i] == null) {
        m_pnpInfo[i] = pnpInfo;
        break;
      }
      // if stored data is older than allowed, invalidate it, and set it to be replaced.
      if (
        m_pnpInfo[i].result().getTimestampSeconds() <=
        Utils.getCurrentTimeSeconds() - INFO_VALID_TIME.in(Seconds)
      ) {
        m_pnpInfo[i] = null;
        indexToReplace = i;
        continue;
      }
      // if selected info has less targets than the provided result
      else if (
        m_pnpInfo[i].result().targets.size() <= pnpInfo.result().targets.size()
      ) {
        // if we have already selected info to replace
        if (indexToReplace > -1) {
          // if we found a worse result, replace that one instead
          if (
            m_pnpInfo[indexToReplace].result().targets.size() >
            m_pnpInfo[i].result().targets.size()
          ) {
            indexToReplace = i;
          }
          // if both results have the same number of targets
          else if (
            m_pnpInfo[indexToReplace].result().targets.size() ==
            m_pnpInfo[i].result().targets.size()
          ) {
            // if (i) was taken later than (indexToReplace), replace (i) instead
            if (
              m_pnpInfo[indexToReplace].result()
                .metadata.captureTimestampMicros >
              m_pnpInfo[i].result().metadata.captureTimestampMicros
            ) {
              indexToReplace = i;
            }
            // if (i) has worse ambiguity than (indexToReplace), replace (i) instead
            else if (
              m_pnpInfo[indexToReplace].result().targets.get(0).poseAmbiguity <
              m_pnpInfo[i].result().targets.get(0).poseAmbiguity
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
    // if an index has been selected to replace, replace it
    if (indexToReplace != -1) m_pnpInfo[indexToReplace] = pnpInfo;
  }

  public void setPrimaryStrategy(PoseStrategy newStrategy) {
    m_primaryStrat = newStrategy;
    m_poseEstimator.setPrimaryStrategy(newStrategy);
  }

  public void setFallbackStrategy(PoseStrategy newStrategy) {
    m_fallbackStrat = newStrategy;
    m_poseEstimator.setMultiTagFallbackStrategy(newStrategy);
  }

  public EstimatedRobotPose getLastEstimatedPose() {
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
    PhotonPipelineResult result,
    PhotonVisionCamera camera
  ) {
    if (camera == null || result == null || !result.hasTargets()) return;

    storePNPInfo(createPNPInfo(result, camera));
    if (result.getBestTarget().objDetectId != -1) {
      cacheForGamepeices(result.targets, camera);
    } else {
      cacheForAprilTags(result.targets, camera);
      calculateBranchPositions();
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
    // double frameTimeSeconds = Utils.fpgaToCurrentTime(
    //   result.getTimestampSeconds()
    // );
    // double currTimeSeconds = Utils.getCurrentTimeSeconds();
    // // if result is older than allowed, do not store it
    // if (frameTimeSeconds <= currTimeSeconds - INFO_VALID_TIME.in(Seconds)) {
    //   return null;
    // }
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

  /**
   * Returns a field relative pose of a target based on where the robot thinks it is, and the provided camera transforms
   * @param targetId The targets fiducial ID
   * @param timeoutMs The amount of milliseconds past which the target information is deemed expired
   * @return Returns the desired targets field relative pose, <strong> will return null if cached data was invalid. </strong>
   */
  public Pose3d getFieldRelativeTargetPose(int targetId, long timeoutMs) {
    return isValidTarget(targetId, timeoutMs)
      ? m_aprilTagInfo[targetId].targetFieldRelativePose
      : null;
  }

  private void calculateBranchPositions() {
    int[] tagsToUse = Robot.alliance == Alliance.Blue
      ? BLUE_REEF_TAGS
      : RED_REEF_TAGS;
    // blues lowest tag starts on side 6 (branch K & branch L)
    // reds lowest tag starts on side 2 (branch C & branch D)
    // so we need this offset to give us our index number to use.
    int reefSideOffset = Robot.alliance == Alliance.Blue ? 5 : 1;
    for (int i = 0; i < tagsToUse.length; i++) {
      Pair<Pose2d, Pose2d> branchPoses = calculateBranchPose(
        m_aprilTagInfo[tagsToUse[i]]
      );
      if (
        branchPoses == null ||
        branchPoses.getFirst() == null ||
        branchPoses.getSecond() == null
      ) continue;
      // i corresponds to the side of the reef were finding the poses of
      // 12 branches, with 6 sides of the reef. each reef has 6 tags on it, and we find which side were calculating for above.
      // for the right branch, we use reefSide * 2 to get its number, since it will always be the higher branch.
      // we do the same for the left branchs, but we subtract one, since its the branch before the right branch.
      int reefSide = ((i + reefSideOffset) % 6) + 1;
      int leftBranchIndex = reefSide * 2 - 1;
      int rightBranchIndex = reefSide * 2;
      Pose2d rightBranchPose = new Pose2d(
        branchPoses.getSecond().getTranslation(),
        REEF_BRANCHES[rightBranchIndex - 1].getRotation()
      );
      rightBranchPose = rightBranchPose.transformBy(
        new Transform2d(
          ROBOT_CONFIGURATION.FULL_LENGTH.div(2)
            .minus(Units.Inches.of(0.25))
            .in(Meters),
          0,
          Rotation2d.kZero
        )
      );
      Pose2d leftBranchPose = new Pose2d(
        branchPoses.getFirst().getTranslation(),
        REEF_BRANCHES[leftBranchIndex - 1].getRotation()
      );
      leftBranchPose = branchPoses
        .getFirst()
        .plus(
          new Transform2d(
            ROBOT_CONFIGURATION.FULL_LENGTH.div(2).in(Meters),
            0,
            Rotation2d.kZero
          )
        );

      m_branchPositions[rightBranchIndex] = rightBranchPose;
      m_branchPositions[leftBranchIndex] = leftBranchPose;
    }

    // the last branch in the BRANCH_GOAL enum is whatever the closest one is, so were assigning that here
    m_branchPositions[0] = Pose2d.kZero;
    m_branchPositions[0] = Robot.swerve
      .getFieldRelativePose2d()
      .nearest(Arrays.asList(m_branchPositions));
  }

  private Pair<Pose2d, Pose2d> calculateBranchPose(TargetInfo targetInfo) {
    if (targetInfo.targetFieldRelativePose == null) {
      return null;
    }
    Pose2d tarPose = targetInfo.targetFieldRelativePose.toPose2d();
    // transform target pose out. The target pose should be in the target coordinate frame, described in PhotonVisions documentation
    Pose2d transformedPose = tarPose.transformBy(
      new Transform2d(
        FIELD_CONSTANTS.BRANCH_TO_TAG_DIST.in(Meters),
        0,
        Rotation2d.kZero
      )
    );
    // branch positions are from the perspective of looking towards the center of the reef
    Pose2d leftBranch = transformedPose.rotateAround(
      tarPose.getTranslation(),
      Rotation2d.kCCW_90deg
    );
    Pose2d rightBranch = transformedPose.rotateAround(
      tarPose.getTranslation(),
      Rotation2d.kCW_90deg
    );
    return new Pair<Pose2d, Pose2d>(leftBranch, rightBranch);
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
   * Returns the calculated branch position of the provided branch
   * @param branchToGet The {@link BRANCH_GOAL} to return the pose of
   * @return The calculated field relative pose of the desired branch
   */
  public Pose2d getFieldRelativeBranchPose(BRANCH_GOAL branchToGet) {
    return m_branchPositions[branchToGet.branchNum - 1];
  }

  public Pose2d getAllianceRelativeBranchPose(BRANCH_GOAL branchToGet) {
    return Robot.alliance == Alliance.Blue
      ? m_branchPositions[branchToGet.branchNum]
      : ChoreoAllianceFlipUtil.flip(m_branchPositions[branchToGet.branchNum]);
  }
}
