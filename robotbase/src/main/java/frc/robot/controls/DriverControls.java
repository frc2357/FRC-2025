package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.util.ForceGyroZero;
import frc.robot.controls.util.AxisInterface;
import frc.robot.controls.util.AxisThresholdTrigger;

public class DriverControls {

  private XboxController m_controller;
  private double m_deadband;

  public JoystickButton m_aButton;
  private JoystickButton m_bButton;
  private JoystickButton m_xButton;
  private JoystickButton m_yButton;

  private JoystickButton m_backButton;
  private JoystickButton m_startButton;
  public JoystickButton m_leftBumper;
  public JoystickButton m_rightBumper;

  private AxisThresholdTrigger m_rightTriggerPrime;
  private AxisThresholdTrigger m_rightTriggerShoot;
  private AxisThresholdTrigger m_leftTrigger;

  private POVButton m_upDPad;
  private POVButton m_rightDPad;
  private POVButton m_downDPad;
  private POVButton m_leftDPad;

  public DriverControls(XboxController controller, double deadband) {
    m_controller = controller;
    m_deadband = deadband;

    m_aButton = new JoystickButton(m_controller, Button.kA.value);
    m_bButton = new JoystickButton(m_controller, Button.kB.value);
    m_xButton = new JoystickButton(m_controller, Button.kX.value);
    m_yButton = new JoystickButton(m_controller, Button.kY.value);

    m_backButton = new JoystickButton(m_controller, Button.kBack.value);
    m_startButton = new JoystickButton(m_controller, Button.kStart.value);

    m_leftBumper = new JoystickButton(controller, Button.kLeftBumper.value);
    m_rightBumper = new JoystickButton(controller, Button.kRightBumper.value);

    m_rightTriggerPrime = new AxisThresholdTrigger(
      m_controller,
      Axis.kRightTrigger,
      0.0
    );
    m_rightTriggerShoot = new AxisThresholdTrigger(
      m_controller,
      Axis.kRightTrigger,
      .8
    );
    m_leftTrigger = new AxisThresholdTrigger(
      m_controller,
      Axis.kLeftTrigger,
      0
    );

    m_upDPad = new POVButton(m_controller, 0);
    m_rightDPad = new POVButton(m_controller, 90);
    m_downDPad = new POVButton(m_controller, 180);
    m_leftDPad = new POVButton(m_controller, 270);

    mapControls();
  }

  public double getRightStickYAxis() {
    return m_controller.getRightY();
  }

  public void mapControls() {
    AxisInterface righStickYAxis = () -> {
      return getRightStickYAxis();
    };

    Trigger noLeftBumper = m_leftBumper.negate();

    m_aButton.whileTrue(
      new DriveToPose(new Pose2d(0, 0, new Rotation2d(0, 0)))
    );

    m_backButton.onTrue(new ForceGyroZero());
  }

  public double getX() {
    return -modifyAxis(m_controller.getLeftX());
  }

  public double getY() {
    return -modifyAxis(m_controller.getLeftY());
  }

  public double getLeftTrigger() {
    return m_controller.getLeftTriggerAxis();
  }

  public boolean isLeftTriggerPressed() {
    return m_leftTrigger.getAsBoolean();
  }

  public double getRotation() {
    return -modifyAxis(m_controller.getRightX());
  }

  /** Only for climb, don't use ever unless Nolan says so */
  public double getLeftStickY() {
    return m_controller.getLeftY();
  }

  public double getRightTriggerAxis() {
    return m_controller.getRightTriggerAxis();
  }

  public double getLeftTriggerAxis() {
    return m_controller.getLeftTriggerAxis();
  }

  public double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public double modifyAxis(double value) {
    value = deadband(value, m_deadband);
    // value = Math.copySign(
    //   Math.pow(value, Constants.SWERVE.TRANSLATION_RAMP_EXPONENT),
    //   value
    // );
    return value;
  }
}
