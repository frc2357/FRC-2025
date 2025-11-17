package frc.robot.vision;

import java.util.Optional;

public interface PipelineComponent {
  public Optional<PipelineResult> process(PipelineResult input);
}
