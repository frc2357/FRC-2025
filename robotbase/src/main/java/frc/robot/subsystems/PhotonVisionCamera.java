package frc.robot.subsystems;

import static frc.robot.Constants.PHOTON_VISION.*;

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
import frc.robot.Robot;
import frc.robot.Constants.FIELD_CONSTANTS;

import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import org.photonvision.*;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonParser;
import com.google.gson.stream.JsonReader;


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

  protected PhotonCameraSim m_simCamera;
  private SimCameraProperties m_simProperties;

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
    if(!Robot.isReal){
      m_simProperties = new SimCameraProperties();
      m_simProperties.setAvgLatencyMs(60);
      m_simProperties.setFPS(30);
      m_simProperties.setExposureTimeMs(30);
      String relativeFilePath = CALIBRATION_FOLDER_PATH+cameraName+"\\"+cameraName+"1280x960Calib.json";
      var camIntrinMatrix = getCameraIntrinsicsFromJson(relativeFilePath);
      var distCoeffMatrix = getDistCoeffsFromJson(relativeFilePath);
      if(camIntrinMatrix != null && distCoeffMatrix != null){
        m_simProperties.setCalibration(1280, 960, camIntrinMatrix, distCoeffMatrix);
        System.out.println("[PhotonVisionCamera] | " + cameraName + " | Sim camera props set successfully.");
      }
      
      m_simCamera = new PhotonCameraSim(m_camera, m_simProperties, FIELD_CONSTANTS.APRIL_TAG_LAYOUT);
      m_simCamera.setMaxSightRange(6);
      // m_simCamera.setMinTargetAreaPercent(2);
      m_simCamera.enableProcessedStream(true);
      m_simCamera.enableDrawWireframe(true);
    }
    else {
      m_simProperties = null;
      m_simCamera = null;
    }
  }

  /**
   * Fetches the latest pipeline result for this instance
   */
  protected void updateResult() {
    if (!m_camera.isConnected() && Robot.isReal) return;

    List<PhotonPipelineResult> results = Robot.isReal ? m_camera.getAllUnreadResults() : 
      List.of(m_simCamera.process(60, new Pose3d(Robot.swerve.getFieldRelativePose2d()).plus(m_robotToCameraTranform), SIM_TARGETS));

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
    return m_camera.isConnected() || !Robot.isReal;
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

  private Matrix<N3,N3> getCameraIntrinsicsFromJson(String relativeFilePath){
    if(Robot.isReal) return null; // If the robot is real, attempting this will throw an exception, so we bail early.
    try {
      var canonicalFilePath = new File(relativeFilePath).getCanonicalPath();
      Map<String,JsonElement> calibJsonMap = JsonParser.parseReader(new JsonReader(new FileReader(canonicalFilePath))).getAsJsonObject().asMap();
      Map<String,JsonElement> camIntrinMap = calibJsonMap.get("cameraIntrinsics").getAsJsonObject().asMap();
      JsonArray camIntrinJsonArray = camIntrinMap.get("data").getAsJsonArray();
      ArrayList<Double> camIntrinValList = new ArrayList<Double>();
      for(JsonElement e : camIntrinJsonArray.asList()){
        camIntrinValList.add(e.getAsDouble());
      }
      Matrix<N3,N3> camIntrinMatrix = new Matrix<N3,N3>(Nat.N3(), Nat.N3());
      for (int i = 0; i < 9; i++) {
        camIntrinMatrix.set(i/3, i%3, camIntrinValList.get(i));
      }
      System.out.println("[PhotonVisionCamera] Succesfully retreived cameraIntrinsics from JSON file.");
      return camIntrinMatrix;
    } catch (Exception e) {
      System.out.println("Getting camera intrinsics from Json failed. Returning null.");
    }
    return null;
  }

  private Matrix<N8,N1> getDistCoeffsFromJson(String relativeFilePath){
    if(Robot.isReal) return null; // If the robot is real, attempting this will throw an exception, so we bail early.
    try {
      var canonicalFilePath = new File(relativeFilePath).getCanonicalPath();
      Map<String,JsonElement> calibJsonMap = JsonParser.parseReader(new JsonReader(new FileReader(canonicalFilePath))).getAsJsonObject().asMap();
      Map<String,JsonElement> distCoeffMap = calibJsonMap.get("distCoeffs").getAsJsonObject().asMap();
      JsonArray distCoeffJsonArray = distCoeffMap.get("data").getAsJsonArray();
      ArrayList<Double> distCoeffValList = new ArrayList<Double>();
      for(JsonElement e : distCoeffJsonArray.asList()){
        distCoeffValList.add(e.getAsDouble());
      }
      Matrix<N8,N1> distCoeffMatrix = new Matrix<N8,N1>(Nat.N8(), Nat.N1());
      for (int i = 0; i < 8; i++) {
        distCoeffMatrix.set(i, 0, distCoeffValList.get(i));
      }
      System.out.println("[PhotonVisionCamera] Succesfully retreived distCoeffs from JSON file.");
      return distCoeffMatrix;
    } catch (Exception e) {
      System.out.println("Getting dist coeffs from Json failed. Returning null.");
    }
    return null;
  }
}
