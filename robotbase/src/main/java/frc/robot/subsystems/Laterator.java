package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.DIGITAL_INPUT;
import frc.robot.Constants.LATERATOR;

public class Laterator extends SubsystemBase {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;

  private DigitalInput m_hallEffectSensor;

  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;

  private Angle m_targetRotations = Units.Rotations.of(Double.NaN);

  public Laterator() {
    m_motorLeft = new SparkMax(
      CAN_ID.LATERATOR_MOTOR_LEFT,
      MotorType.kBrushless
    );

    m_motorRight = new SparkMax(
      CAN_ID.LATERATOR_MOTOR_RIGHT,
      MotorType.kBrushless
    );

    m_motorLeft.configure(
      LATERATOR.MOTOR_CONFIG_LEFT,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_motorRight.configure(
      LATERATOR.MOTOR_CONFIG_RIGHT,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_PIDController = m_motorLeft.getClosedLoopController();

    m_encoder = m_motorLeft.getEncoder();

    m_hallEffectSensor = new DigitalInput(
      DIGITAL_INPUT.LATERATOR_CENTER_HALL_EFFECT_SENSOR_ID
    );
  }

  public void setTargetRotations(Angle targetRotations) {
    m_targetRotations = targetRotations;
    m_PIDController.setReference(
      m_targetRotations.in(Units.Rotations),
      ControlType.kMAXMotionPositionControl
    );
  }

  public Angle getRotations() {
    return Units.Rotations.of(m_encoder.getPosition());
  }

  public boolean isAtTargetRotations() {
    return m_targetRotations.isNear(
      getRotations(),
      LATERATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT
    );
  }

  public boolean isAtSetpoint() {
    return isAtTargetRotations();
  }

  public boolean isAtCenter() {
    return m_hallEffectSensor.get();
  }

  public AngularVelocity getVelocity() {
    return Units.RotationsPerSecond.of(m_encoder.getVelocity() / 60);
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  public void setSpeed(double speed) {
    m_motorLeft.set(speed);
    setTargetRotations(Units.Rotations.of(Double.NaN));
  }

  public void setAxisSpeed(double axisSpeed) {
    axisSpeed *= LATERATOR.AXIS_MAX_SPEED;
    m_motorLeft.set(axisSpeed);
    setTargetRotations(Units.Rotations.of(Double.NaN));
  }

  public void stop() {
    m_motorLeft.stopMotor();
    setTargetRotations(Units.Rotations.of(Double.NaN));
  }
}
