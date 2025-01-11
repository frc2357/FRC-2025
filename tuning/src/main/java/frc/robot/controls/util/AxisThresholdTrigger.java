package frc.robot.controls.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AxisThresholdTrigger extends Trigger {

  public AxisThresholdTrigger(XboxController controller, Axis axis, double triggerThreshold) {
    super(() -> controller.getRawAxis(axis.value) > triggerThreshold);
  }
}
