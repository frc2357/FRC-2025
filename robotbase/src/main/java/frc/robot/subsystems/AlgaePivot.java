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

  public AlgaePivot() {
    m_leftMotor = new SparkMax(
      CAN_ID.ALGAE_PIVOT_LEFT_MOTOR,
      MotorType.kBrushless
    );
    m_rightMotor = new SparkMax(
      CAN_ID.ALGAE_PIVOT_RIGHT_MOTOR,
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
  }

  public void setSpeed(double percentOutput) {
    m_rightMotor.set(percentOutput);
  }

  public void setAxisSpeed(double axisSpeed) {
    axisSpeed *= ALGAE_PIVOT.AXIS_MAX_SPEED;
    setSpeed(axisSpeed);
  }

  public boolean isStalling() {
    return (m_rightMotor.getOutputCurrent() >= ALGAE_PIVOT.STALL_CURRENT_VOLTS);
  }

  public void stop() {
    m_rightMotor.stopMotor();
  }
}
