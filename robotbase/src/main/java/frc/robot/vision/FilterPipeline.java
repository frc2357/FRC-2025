package frc.robot.vision;

import java.util.Collections;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class FilterPipeline implements Runnable {

  private List<PipelineComponent> m_components;
  private PhotonCamera m_camera;
  private PhotonPoseEstimator m_estimator;

  public FilterPipeline(PhotonCamera camera, PhotonPoseEstimator estimator) {
    m_camera = camera;
    m_estimator = estimator;
  }

  public void withComponents(PipelineComponent... components) {
    Collections.addAll(m_components, components);
  }

  @Override
  public void run() {
    List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
    for (PhotonPipelineResult result : results) {
      Optional<EstimatedRobotPose> estimate = m_estimator.update(result);

      if (estimate.isPresent()) {} else {
        // Logging maybe?
      }
    }
  }

  public Optional<PipelineResult> processPipeline(EstimatedRobotPose estimate) {
    PipelineResult currentEstimate = new PipelineResult(estimate);
    return processPipeline(currentEstimate);
  }

  public Optional<PipelineResult> processPipeline(PipelineResult estimate) {
    Optional<PipelineResult> currentResult = Optional.ofNullable(estimate);
    for (PipelineComponent component : m_components) {
      if (currentResult.isPresent()) {
        currentResult = component.process(currentResult.get());
      }
    }

    return currentResult;
  }
}
