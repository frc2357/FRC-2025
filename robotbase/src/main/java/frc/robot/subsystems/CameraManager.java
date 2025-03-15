package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.PHOTON_VISION.*;

import com.ctre.phoenix6.Utils;
import com.google.errorprone.annotations.DoNotCall;
import com.google.errorprone.annotations.DoNotMock;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Constants.PHOTON_VISION;
import frc.robot.Robot;
import frc.robot.subsystems.PhotonVisionCamera.TimestampedPNPInfo;
import frc.robot.util.CollisionDetection;
import java.util.ArrayList;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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
        averageDistBetweenEstimates > PHOTON_VISION.MAX_DIST_BETWEEN_ESTIMATES
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
    m_poseEstimator.setMultiTagFallbackStrategy(m_fallbackStrat);
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
      this::storePNPInfo
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
  }

  private poseEstimate estimatePoseWithPNPInfo(TimestampedPNPInfo pnpInfo) {
    if (pnpInfo == null || !pnpInfo.exists()) return null;
    preparePoseEstimator(pnpInfo);
    EstimatedRobotPose estimate = m_poseEstimator
      .update(
        pnpInfo.result(),
        pnpInfo.camMatrix(),
        pnpInfo.distCoeefs(),
        POSE_EST_PARAMS
      )
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
      }
      // and we have updated the pose recently, and were not disabled, throw it out
      if (
        measuredTime <
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
        Utils.getCurrentTimeSeconds() - PNP_INFO_VALID_TIME.in(Seconds)
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
}
