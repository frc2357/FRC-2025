package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.DIGITAL_INPUT;
import frc.robot.Constants.ELEVATOR;
import frc.robot.util.Utility;

public class Elevator extends SubsystemBase {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;

  private DigitalInput m_hallEffectSensor;
  private Debouncer m_debouncer;
  private boolean m_isAtZero = false;

  private MutAngle m_targetRotations = Units.Rotations.mutable(Double.NaN);
  private MutAngularVelocity m_currentAngularVelocityHolder = Units.RPM.mutable(
    Double.NaN
  );
  private MutAngle m_currentRotationsHolder = Units.Rotations.mutable(
    Double.NaN
  );

  public Elevator() {
    SmartDashboard.putNumber("Elevator Setpoint Modifier", 0);
    m_motorLeft = new SparkMax(
      CAN_ID.ELEVATOR_LEFT_MOTOR,
      MotorType.kBrushless
    );
    m_motorRight = new SparkMax(
      CAN_ID.ELEVATOR_RIGHT_MOTOR,
      MotorType.kBrushless
    );

    m_motorLeft.configure(
      ELEVATOR.MOTOR_CONFIG_LEFT,
      ResetMode.kResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
    m_motorRight.configure(
      ELEVATOR.MOTOR_CONFIG_RIGHT,
      ResetMode.kResetSafeParameters,
      PersistMode.kNoPersistParameters
    );

    m_PIDController = m_motorLeft.getClosedLoopController();

    m_encoder = m_motorLeft.getEncoder();
    m_hallEffectSensor = new DigitalInput(
      DIGITAL_INPUT.ELEVATOR_HALL_EFFECT_SENSOR_ID
    );
    m_debouncer = new Debouncer(ELEVATOR.DEBOUNCE_TIME_SECONDS);
  }

  public void setSpeed(double percentOutput) {
    m_motorLeft.set(percentOutput);
    m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
  }

  public void setAxisSpeed(double speed) {
    speed *= ELEVATOR.AXIS_MAX_SPEED;
    m_motorLeft.set(speed);
    m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
  }

  public void stop() {
    m_motorLeft.stopMotor();
    m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
  }

  @SuppressWarnings("removal")
  private void setTargetRotations(Angle targetRotations) {
    m_targetRotations.mut_replace(targetRotations);
    m_PIDController.setReference(
      m_targetRotations.in(Units.Rotations),
      ControlType.kSmartMotion,
      ClosedLoopSlot.kSlot0,
      ELEVATOR.LEFT_MOTOR_ARB_F,
      ArbFFUnits.kVoltage
    );
  }

  public void setTargetDistance(Distance targetDistance) {
    setTargetRotations(distanceToRotations(targetDistance));
  }

  public void holdPosition() {
    m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
    m_motorLeft.setVoltage(ELEVATOR.HOLD_VOLTAGE);
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
    return rotationsToDistance(getRotations());
  }

  private boolean isAtTargetRotations() {
    return Utility.isWithinTolerance(
      getRotations(),
      m_targetRotations,
      ELEVATOR.SMART_MOTION_ALLOWED_ERROR_ROTATIONS
    );
  }

  public boolean isAtTarget() {
    return isAtTargetRotations();
  }

  public boolean isGoingDown() {
    return m_targetRotations.lt(getRotations());
  }

  public boolean isStalling() {
    return m_motorLeft.getOutputCurrent() > ELEVATOR.ZERO_STALL_AMPS;
  }

  public boolean isAtZero() {
    return m_isAtZero;
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  public void setPositionHallEffect() {
    m_encoder.setPosition(
      distanceToRotations(ELEVATOR.SETPOINTS.HALL_EFFECT_POSITION).in(
        Units.Rotations
      )
    );
  }

  private Angle distanceToRotations(Distance distance) {
    return Units.Rotations.of(
      distance
        .div(ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE)
        .times(ELEVATOR.GEAR_RATIO)
        .magnitude()
    );
  }

  private Distance rotationsToDistance(Angle rotations) {
    return (
      ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE.times(
        rotations.div(ELEVATOR.GEAR_RATIO).in(Units.Rotations)
      )
    );
  }

  public void updateSensors() {
    m_isAtZero = m_debouncer.calculate(!m_hallEffectSensor.get());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
      "Elevator Calculated Distance",
      getDistance().in(Units.Inches)
    );

    SmartDashboard.putBoolean("Hall Effect", isAtZero());
  }
}
