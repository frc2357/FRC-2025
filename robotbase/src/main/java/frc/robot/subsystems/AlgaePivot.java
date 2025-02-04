package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ALGAE_PIVOT;
import frc.robot.Constants.CAN_ID;

public class AlgaePivot extends SubsystemBase {

  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;
  private SparkAbsoluteEncoder m_absoluteEncoder;
  private SparkClosedLoopController m_leftPidController;

  private MutAngle m_targetAngle = Units.Degrees.mutable(Double.NaN);
  private MutAngle m_currentAngleHolder = Units.Degrees.mutable(Double.NaN);

  public AlgaePivot() {
    m_leftMotor = new SparkMax(
      CAN_ID.LEFT_ALGAE_PIVOT_MOTOR,
      MotorType.kBrushless
    );
    m_rightMotor = new SparkMax(
      CAN_ID.RIGHT_ALGAE_PIVOT_MOTOR,
      MotorType.kBrushless
    );
    m_leftMotor.configure(
      ALGAE_PIVOT.LEFT_MOTOR_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    m_rightMotor.configure(
      ALGAE_PIVOT.RIGHT_MOTOR_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    m_absoluteEncoder = m_leftMotor.getAbsoluteEncoder();
    m_leftPidController = m_leftMotor.getClosedLoopController();
  }

  public void setSpeed(double percentOutput) {
    m_leftMotor.set(percentOutput);
    m_targetAngle.mut_replace(Double.NaN, Units.Rotations);
  }

  public void setAxisSpeed(double axisSpeed) {
    axisSpeed *= ALGAE_PIVOT.AXIS_MAX_SPEED;
    setSpeed(axisSpeed);
    m_targetAngle.mut_replace(Double.NaN, Units.Rotations);
  }

  public Angle getAngle() {
    m_currentAngleHolder.mut_replace(
      m_absoluteEncoder.getPosition(),
      Units.Rotations
    );
    return m_currentAngleHolder;
  }

  public void stop() {
    m_leftMotor.stopMotor();
    m_targetAngle.mut_replace(Double.NaN, Units.Rotations);
  }

  public void setTargetAngle(Angle angle) {
    if (angle.lt(ALGAE_PIVOT.MIN_ANGLE)) {
      System.err.println("ALGAE PIVOT: Cannot set angle lower than minimum");
      return;
    }
    if (angle.gt(ALGAE_PIVOT.MAX_ANGLE)) {
      System.err.println("ALGAE PIVOT: Cannot set angle higher than minimum");
      return;
    }

    m_targetAngle.mut_replace(angle);
    m_leftPidController.setReference(
      angle.in(Units.Rotations),
      ControlType.kPosition
    );
  }

  public Angle getTargetAngle() {
    return m_targetAngle;
  }

  public boolean isAtTarget() {
    return m_targetAngle.isNear(
      getAngle(),
      ALGAE_PIVOT.MAX_MOTION_ALLOWED_ERROR_PERCENT
    );
  }
}
