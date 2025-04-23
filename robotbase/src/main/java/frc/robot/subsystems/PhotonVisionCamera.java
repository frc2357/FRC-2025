package frc.robot.subsystems;

import static frc.robot.Constants.PHOTON_VISION.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.MerweScaledSigmaPoints;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.Constants.FIELD_CONSTANTS;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;

/** Controls the photon vision camera options. */
public class PhotonVisionCamera {

  public final record TimestampedPNPInfo(
    PhotonPipelineResult result,
    Rotation3d heading,
    double headingTimestampSeconds,
    Transform3d robotToCameraTransform,
    PhotonVisionCamera camera
  ) {
    public boolean exists() {
      return (
        result != null &&
        heading != null &&
        !Double.isNaN(headingTimestampSeconds) &&
        robotToCameraTransform != null &&
        camera != null
      );
    }
  }

  protected final PhotonCamera m_camera;

  protected final Transform3d m_robotToCameraTranform;

  protected final NetworkTable m_poseOutputTable;
  protected final DoubleArrayPublisher m_poseFieldPub;
  protected final StringPublisher m_poseFieldTypePub;

  protected final NetworkTable m_photonTable;
  protected final DoubleArraySubscriber m_intrinSub;
  protected final DoubleArraySubscriber m_distortSub;

  protected PhotonPipelineResult m_result;

  private final BiConsumer<
    PhotonVisionCamera,
    PhotonPipelineResult
  > m_resultProccessor;

  protected int m_bestTargetFiducialId;

  protected final PhotonPoseEstimator m_poseEstimator;

  // experimental "turbo switch" that has the ability to increase FPS. Do not fiddle with it.
  @SuppressWarnings("unused")
  private static final BooleanEntry m_turboSwitch =
    NetworkTableInstance.getDefault()
      .getBooleanTopic("/photonvision/use_new_cscore_frametime")
      .getEntry(ACTIVATE_TURBO_SWITCH);

  public PhotonVisionCamera(
    String cameraName,
    Transform3d robotToCameraTransform,
    BiConsumer<PhotonVisionCamera, PhotonPipelineResult> resultConsumer
  ) {
    this(
      cameraName,
      robotToCameraTransform,
      Optional.empty(),
      Optional.empty(),
      resultConsumer
    );
  }

  public PhotonVisionCamera(
    String cameraName,
    Transform3d robotToCameraTransform,
    Optional<Matrix<N3, N3>> camMatrix,
    Optional<Matrix<N8, N1>> distCoefs,
    BiConsumer<PhotonVisionCamera, PhotonPipelineResult> resultConsumer
  ) {
    m_camera = new PhotonCamera(cameraName);

    m_poseOutputTable = NetworkTableInstance.getDefault()
      .getTable("VisionPose-" + cameraName);
    m_poseFieldPub = m_poseOutputTable.getDoubleArrayTopic("pose").publish();
    m_poseFieldTypePub = m_poseOutputTable.getStringTopic(".type").publish();

    m_photonTable = NetworkTableInstance.getDefault()
      .getTable("photonvision")
      .getSubTable(cameraName);
    m_distortSub = m_photonTable
      .getDoubleArrayTopic("cameraDistortion")
      .getEntry(null);
    m_intrinSub = m_photonTable
      .getDoubleArrayTopic("cameraIntrinsics")
      .getEntry(null);

    m_robotToCameraTranform = robotToCameraTransform;
    m_resultProccessor = resultConsumer;
    m_poseEstimator = new PhotonPoseEstimator(
      FIELD_CONSTANTS.APRIL_TAG_LAYOUT,
      PRIMARY_STRATEGY,
      m_robotToCameraTranform
    );
  }

  /**
   * Fetches the latest pipeline result for this instance
   */
  protected void updateResult() {
    if (!m_camera.isConnected()) return;

    List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();

    // no new results, so we stop here.
    if (results.isEmpty()) return;

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

    m_resultProccessor.accept(this, m_result); // update pnpInfo with the new result
  }

  public void publishPose(Pose2d pose) {
    if (pose == null) {
      return;
    }
    m_poseFieldTypePub.set("Field2d");
    m_poseFieldPub.accept(
      new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() }
    );
  }

  public boolean isConnected() {
    return m_camera.isConnected();
  }

  public double getLatencyMillis() {
    return isConnected() ? m_result.metadata.getLatencyMillis() : Double.NaN;
  }

  public double getTimestampSeconds() {
    return isConnected() ? m_result.getTimestampSeconds() : Double.NaN;
  }

  public void setPipeline(int index) {
    if (m_camera.getPipelineIndex() != index) {
      m_camera.setPipelineIndex(index);
    }
  }

  public int getPipeline() {
    return m_camera.getPipelineIndex();
  }

  public int numberOfTargetsSeen() {
    return hasTarget() ? m_result.targets.size() : -1;
  }

  public boolean hasTarget() {
    return m_result != null && m_result.hasTargets();
  }

  public Transform3d getBestTargetRobotRelativeTransform() {
    return m_result != null && m_result.hasTargets()
      ? m_result
        .getBestTarget()
        .bestCameraToTarget.plus(m_robotToCameraTranform)
      : null;
  }

  public String getName() {
    return m_camera.getName();
  }
}
