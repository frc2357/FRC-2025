package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.PHOTON_VISION.*;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;
import frc.robot.commands.util.InitCamera;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Controls the photon vision camera options. */
public class PhotonVisionCamera {

  public final record TimestampedPNPInfo(
    PhotonPipelineResult result,
    Rotation3d heading,
    double headingTimestampSeconds,
    Optional<Matrix<N3, N3>> camMatrix,
    Optional<Matrix<N8, N1>> distCoeefs,
    Transform3d robotToCameraTransform,
    PhotonVisionCamera camera
  ) {
    public boolean exists() {
      return (
        result != null &&
        heading != null &&
        !Double.isNaN(headingTimestampSeconds) &&
        (camMatrix != null && camMatrix.isPresent()) &&
        (distCoeefs != null && distCoeefs.isPresent()) &&
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

  private Optional<Matrix<N3, N3>> m_cameraMatrix;
  private Optional<Matrix<N8, N1>> m_distCoefs;

  private final Consumer<TimestampedPNPInfo> m_pnpInfoStorer;

  /**
   * The fiducial ID of the best target we have.
   *
   * <p>Used for methods that dont take in a fid ID but do some april tag stuff.
   */
  // protected int m_bestTargetFiducialId;

  // experimental "turbo switch" that has the ability to increase FPS. Do not fiddle with it.
  @SuppressWarnings("unused")
  private static final BooleanEntry m_turboSwitch =
    NetworkTableInstance.getDefault()
      .getBooleanTopic("/photonvision/use_new_cscore_frametime")
      .getEntry(ACTIVATE_TURBO_SWITCH);

  public PhotonVisionCamera(
    String cameraName,
    Transform3d robotToCameraTransform,
    Consumer<TimestampedPNPInfo> infoConsumer
  ) {
    this(
      cameraName,
      robotToCameraTransform,
      Optional.empty(),
      Optional.empty(),
      infoConsumer
    );
  }

  public PhotonVisionCamera(
    String cameraName,
    Transform3d robotToCameraTransform,
    Optional<Matrix<N3, N3>> camMatrix,
    Optional<Matrix<N8, N1>> distCoefs,
    Consumer<TimestampedPNPInfo> infoConsumer
  ) {
    m_camera = new PhotonCamera(cameraName);

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
    m_distortSub = m_photonTable
      .getDoubleArrayTopic("cameraDistortion")
      .getEntry(null);
    m_intrinSub = m_photonTable
      .getDoubleArrayTopic("cameraIntrinsics")
      .getEntry(null);

    m_robotToCameraTranform = robotToCameraTransform;
    m_cameraMatrix = camMatrix;
    m_distCoefs = distCoefs;
    if (camMatrix.isEmpty() || distCoefs.isEmpty()) {
      m_cameraMatrix = m_camera.getCameraMatrix();
      m_distCoefs = m_camera.getDistCoeffs();
      if (camMatrix.isEmpty() || distCoefs.isEmpty()) new InitCamera(
        this
      ).schedule();
    }
    m_pnpInfoStorer = infoConsumer;
  }

  public boolean getDistCoefs() {
    double[] distArray = m_distortSub.get();
    if (distArray == null) {
      return false;
    }
    Matrix<N8, N1> distCoefs = new Matrix<N8, N1>(
      Nat.N8(),
      Nat.N1(),
      distArray
    );
    m_distCoefs = Optional.of(distCoefs);
    return true;
  }

  public boolean getCameraMatrix() {
    double[] intrinsicsArray = m_intrinSub.get();
    if (intrinsicsArray == null) return false;
    Matrix<N3, N3> cameraMatrix = new Matrix<N3, N3>(
      Nat.N3(),
      Nat.N3(),
      intrinsicsArray
    );
    m_cameraMatrix = Optional.of(cameraMatrix);
    return true;
  }

  /**
   * Fetches the latest pipeline result for this instance
   */
  protected void updateResult() {
    if (!m_camera.isConnected()) return;

    List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();

    if (results.isEmpty()) { // no new results, so we stop here.
      return;
    }
    if (m_caching) {
      processAprilTagResult(results);
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

    m_pnpInfoStorer.accept(createPNPInfo(m_result)); // update pnpInfo with the new result
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

  /**
   * Updates the {@link #m_pnpInfo} list with a result, and replaces the worst result, if the provided result is better.
   *
   * @param result The provided {@link PhotonPipelineResult result} to use for updating the pnpInfo list
   */
  private TimestampedPNPInfo createPNPInfo(PhotonPipelineResult result) {
    // if the provided result has no targets, it has no value, so we do not want to store it
    if (result == null || !result.hasTargets()) {
      return null;
    }
    double frameTimeSeconds = Utils.fpgaToCurrentTime(
      result.getTimestampSeconds()
    );
    double currTimeSeconds = Utils.getCurrentTimeSeconds();
    // if result is older than allowed, do not store it
    if (frameTimeSeconds <= currTimeSeconds - PNP_INFO_VALID_TIME.in(Seconds)) {
      return null;
    }
    // if rotating too fast, dont create info
    if (
      Math.abs(Robot.swerve.getAngularVelocity().in(RadiansPerSecond)) >
      MAX_ACCEPTABLE_ROTATIONAL_VELOCITY.in(RadiansPerSecond)
    ) return null;
    // if translating too fast, dont create info
    if (
      Math.abs(
        Robot.swerve.getAbsoluteTranslationalVelocity().in(MetersPerSecond)
      ) >
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
      m_cameraMatrix,
      m_distCoefs,
      m_robotToCameraTranform,
      this
    );
  }

  // private void cacheForGamepeices(List<PhotonTrackedTarget> targetList) {
  //   PhotonTrackedTarget bestTarget = calculateBestGamepeiceTarget(targetList);
  //   TargetInfo aprilTagInfo = m_aprilTagInfo[0];
  //   aprilTagInfo.yaw = bestTarget.getYaw();
  //   aprilTagInfo.pitch = bestTarget.getPitch();
  //   aprilTagInfo.timestamp = m_result.metadata.captureTimestampMicros;
  // }

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

  public boolean isConnected() {
    return m_camera.isConnected();
  }

  public double getLatencyMillis() {
    return isConnected() ? m_result.metadata.getLatencyMillis() : Double.NaN;
  }

  public double getTimestampSeconds() {
    return isConnected() ? m_result.getTimestampSeconds() : Double.NaN;
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

  public int getPipeline() {
    return m_camera.getPipelineIndex();
  }

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

  public int numberOfTargetsSeen() {
    return m_result.targets.size();
  }

  public boolean hasTarget() {
    return m_result.hasTargets();
  }

  public String getName() {
    return m_camera.getName();
  }

  //*********************** Methods for traditional april tag tracking and caching *************************************/
  // Separating methods like this is not a good pattern FYI

  /**
   * The method to cache target data for AprilTags.
   *
   * @param targetList The list of targets that it pulls data from to cache.
   */

  /*
   * The class for the object we use to cache our target data
   */
  private static class TargetInfo {

    public double yaw = Double.NaN;
    public double pitch = Double.NaN;
    public long timestamp = 0;
  }

  /**
   * The list of TargetInfo objects where we cache all of the target data.
   */
  private TargetInfo[] m_aprilTagInfo;

  /**
   * The fiducial ID of the best target we have.
   *
   * <p>Used for methods that dont take in a fid ID but do some april tag stuff.
   */
  private int m_bestTargetFiducialId;

  private boolean m_caching = false;

  public PhotonVisionCamera(
    String cameraName,
    Transform3d robotToCameraTransform,
    Optional<Matrix<N3, N3>> camMatrix,
    Optional<Matrix<N8, N1>> distCoefs,
    Consumer<TimestampedPNPInfo> infoConsumer,
    boolean caching
  ) {
    this(
      cameraName,
      robotToCameraTransform,
      camMatrix,
      distCoefs,
      infoConsumer
    );
    m_aprilTagInfo = new TargetInfo[22];
    for (int i = 0; i < m_aprilTagInfo.length; i++) {
      m_aprilTagInfo[i] = new TargetInfo();
    }
  }

  private void processAprilTagResult(List<PhotonPipelineResult> results) {
    boolean foundBest = false;
    for (PhotonPipelineResult result : results) {
      if (result == null || !result.hasTargets()) {
        continue;
      }
      if (!foundBest) {
        m_bestTargetFiducialId = m_result.getBestTarget().getFiducialId();
        foundBest = true;
      }
      cacheForAprilTags(result.targets);
    }
  }

  private void cacheForAprilTags(List<PhotonTrackedTarget> targetList) {
    long now = System.currentTimeMillis();
    for (PhotonTrackedTarget targetSeen : targetList) {
      int id = targetSeen.getFiducialId();
      TargetInfo targetInfo = m_aprilTagInfo[id];
      // System.out.println(targetSeen.getFiducialId());
      targetInfo.yaw = targetSeen.getYaw();
      targetInfo.pitch = targetSeen.getPitch();
      targetInfo.timestamp = now;
    }
  }

  /**
   * @param fiducialId The fiducial ID of the target to get the yaw of.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the desired targets yaw. <strong>Will be null if the cached data was invalid.
   */
  public Optional<Angle> getTargetYaw(int targetId, long timeoutMs) {
    if (isValidTarget(targetId, timeoutMs)) {
      return Optional.of(Units.Degrees.of(m_aprilTagInfo[targetId].yaw));
    }
    return Optional.empty();
  }

  /**
   * @param id The ID of the target to get the pitch of.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the desired targets pitch, <strong>will be null if the cached data was invalid.
   */
  public Optional<Angle> getTargetPitch(int targetId, long timeoutMs) {
    if (isValidTarget(targetId, timeoutMs)) {
      return Optional.of(Units.Degrees.of(m_aprilTagInfo[targetId].pitch));
    }
    return Optional.empty();
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
      Math.abs(target.yaw) > MAX_ANGLE.in(Degrees) ||
      Math.abs(target.pitch) > MAX_ANGLE.in(Degrees)
    );
  }

  /**
   * Gets what PhotonVision said the best target was last time it looked.
   *
   * @return The fiducial id of the best target
   */
  public int getBestTargetFiducialId() {
    return m_bestTargetFiducialId;
  }
}
