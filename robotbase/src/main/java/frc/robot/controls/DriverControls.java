package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.controls.util.AxisInterface;
import frc.robot.controls.util.AxisThresholdTrigger;

@SuppressWarnings("unused")
public class DriverControls {

  private CommandXboxController m_controller;

  private double m_deadband;

  public Trigger m_aButton;
  private Trigger m_bButton;
  private Trigger m_xButton;
  private Trigger m_yButton;

  private Trigger m_backButton;
  private Trigger m_startButton;
  public Trigger m_leftBumper;
  public Trigger m_rightBumper;

  private AxisThresholdTrigger m_rightTrigger;
  private AxisThresholdTrigger m_leftTrigger;

  private Trigger m_upDPad;
  private Trigger m_rightDPad;
  private Trigger m_downDPad;
  private Trigger m_leftDPad;

  public DriverControls(CommandXboxController controller, double deadband) {
    m_controller = controller;
    m_deadband = deadband;

    m_aButton = m_controller.a();
    m_bButton = m_controller.b();
    m_xButton = m_controller.x();
    m_yButton = m_controller.y();

    m_backButton = m_controller.back();
    m_startButton = m_controller.start();

    m_leftBumper = m_controller.leftBumper();
    m_rightBumper = m_controller.rightBumper();

    m_rightTrigger = new AxisThresholdTrigger(
      m_controller,
      Axis.kRightTrigger,
      0.0
    );
    m_leftTrigger = new AxisThresholdTrigger(
      m_controller,
      Axis.kLeftTrigger,
      0
    );

    m_upDPad = m_controller.povUp();
    m_rightDPad = m_controller.povRight();
    m_downDPad = m_controller.povDown();
    m_leftDPad = m_controller.povLeft();

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

    m_startButton.onTrue(
      new InstantCommand(() -> Robot.swerve.seedFieldCentric())
    );
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
