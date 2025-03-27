package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.CLIMBER_PIVOT;

public class ClimberPivot extends SubsystemBase {

  private SparkMax m_motor;

  public ClimberPivot() {
    m_motor = new SparkMax(CAN_ID.CLIMBER_PIVOT_MOTOR, MotorType.kBrushless);
    m_motor.configure(
      CLIMBER_PIVOT.MOTOR_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void setSpeed(double percentOutput) {
    m_motor.set(percentOutput);
  }

  public void setAxisSpeed(double speed) {
    speed *= CLIMBER_PIVOT.AXIS_MAX_SPEED;
    m_motor.set(speed);
  }

  public void stop() {
    m_motor.stopMotor();
  }
}
