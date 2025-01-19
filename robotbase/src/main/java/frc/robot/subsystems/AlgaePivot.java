package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ALGAE_PIVOT;
import frc.robot.Constants.CAN_ID;

public class AlgaePivot extends SubsystemBase {

  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;

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
  }

  public void setAxisSpeed(double axisSpeed) {
    axisSpeed *= ALGAE_PIVOT.AXIS_MAX_SPEED;
    set(axisSpeed);
  }

  public void set(double percentOutput) {
    m_leftMotor.set(percentOutput);
  }

  public void stop() {
    m_leftMotor.stopMotor();
  }
}
