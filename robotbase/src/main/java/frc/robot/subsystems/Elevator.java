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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;

public class Elevator extends SubsystemBase {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;

  private MutAngle m_targetRotations = Units.Rotations.mutable(Double.NaN);
  private MutAngularVelocity m_currentAngularVelocityHolder = Units.RPM.mutable(
    Double.NaN
  );
  private MutAngle m_currentRotationsHolder = Units.Rotations.mutable(
    Double.NaN
  );

  public Elevator() {
    m_motorLeft = new SparkMax(
      Constants.CAN_ID.ELEVATOR_LEFT_MOTOR,
      MotorType.kBrushless
    );
    m_motorRight = new SparkMax(
      Constants.CAN_ID.ELEVATOR_RIGHT_MOTOR,
      MotorType.kBrushless
    );

    m_motorLeft.configure(
      Constants.ELEVATOR.MOTOR_CONFIG_LEFT,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    m_motorRight.configure(
      Constants.ELEVATOR.MOTOR_CONFIG_RIGHT,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_PIDController = m_motorLeft.getClosedLoopController();

    m_encoder = m_motorLeft.getEncoder();
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

  public void setTargetRotations(Angle targetRotations) {
    m_targetRotations.mut_replace(targetRotations);
    m_PIDController.setReference(
      m_targetRotations.in(Units.Rotations),
      ControlType.kMAXMotionPositionControl,
      ClosedLoopSlot.kSlot0,
      ELEVATOR.LEFT_MOTOR_ARB_F,
      ArbFFUnits.kVoltage
    );
  }

  public void setTargetDistance(Distance targetDistance) {
    Angle rotations = Units.Rotations.of(
      targetDistance
        .div(ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE)
        .times(ELEVATOR.GEAR_RATIO)
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

  public Angle getRotations() {
    m_currentRotationsHolder.mut_replace(
      m_encoder.getPosition(),
      Units.Rotations
    );
    return m_currentRotationsHolder;
  }

  public Distance getDistance() {
    return (
      ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE.times(
        getRotations().div(ELEVATOR.GEAR_RATIO).in(Units.Rotations)
      )
    );
  }

  private boolean isAtTargetRotations() {
    return m_targetRotations.isNear(
      getRotations(),
      ELEVATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT
    );
  }

  public boolean isAtTarget() {
    return isAtTargetRotations();
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Elevator RPM", m_encoder.getVelocity());
  }
}
