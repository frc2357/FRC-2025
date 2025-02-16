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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.DIGITAL_INPUT;
import frc.robot.Constants.LATERATOR;

public class Laterator extends SubsystemBase {

  private SparkMax m_motor;

  private DigitalInput m_hallEffectSensor;

  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;

  private MutAngle m_targetRotations = Units.Rotations.mutable(Double.NaN);
  private MutAngularVelocity m_currentAngularVelocityHolder = Units.RPM.mutable(
    Double.NaN
  );
  private MutAngle m_currentRotationsHolder = Units.Rotations.mutable(
    Double.NaN
  );

  public Laterator() {
    m_motor = new SparkMax(CAN_ID.LATERATOR_MOTOR, MotorType.kBrushless);

    m_motor.configure(
      LATERATOR.MOTOR_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_PIDController = m_motor.getClosedLoopController();

    m_encoder = m_motor.getEncoder();

    m_hallEffectSensor = new DigitalInput(
      DIGITAL_INPUT.LATERATOR_CENTER_HALL_EFFECT_SENSOR_ID
    );
  }

  public void setSpeed(double percentOutput) {
    m_motor.set(percentOutput);
    m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
  }

  public void setAxisSpeed(double axisSpeed) {
    axisSpeed *= LATERATOR.AXIS_MAX_SPEED;
    m_motor.set(axisSpeed);
    m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
  }

  public void stop() {
    m_motor.stopMotor();
    m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
  }

  private void setTargetRotations(Angle targetRotations) {
    m_targetRotations.mut_replace(targetRotations);
    m_PIDController.setReference(
      m_targetRotations.in(Units.Rotations),
      ControlType.kMAXMotionPositionControl
    );
  }

  public void setTargetDistance(Distance targetDistance) {
    Angle rotations = Units.Rotations.of(
      targetDistance
        .div(LATERATOR.OUTPUT_PULLEY_CIRCUMFERENCE)
        .times(LATERATOR.GEAR_RATIO)
        .magnitude()
    );
    setTargetRotations(rotations);
  }

  public AngularVelocity getVelocity() {
    m_currentAngularVelocityHolder.mut_replace(
      m_encoder.getVelocity(),
      Units.RPM
    );
    return m_currentAngularVelocityHolder;
  }

  private Angle getRotations() {
    m_currentRotationsHolder.mut_replace(
      m_encoder.getPosition(),
      Units.Rotations
    );
    return m_currentRotationsHolder;
  }

  public Distance getDistance() {
    return (
      LATERATOR.OUTPUT_PULLEY_CIRCUMFERENCE.times(
        getRotations().div(LATERATOR.GEAR_RATIO).in(Units.Rotations)
      )
    );
  }

  private boolean isAtTargetRotations() {
    return m_targetRotations.isNear(
      getRotations(),
      LATERATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT
    );
  }

  public boolean isAtTarget() {
    return isAtTargetRotations();
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  public boolean isAtZero() {
    return m_hallEffectSensor.get();
  }
}
