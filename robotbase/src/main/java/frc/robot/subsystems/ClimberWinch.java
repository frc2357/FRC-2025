package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.CLIMBER_WINCH;

public class ClimberWinch extends SubsystemBase {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;

  public ClimberWinch() {
    m_motorLeft = new SparkMax(
      CAN_ID.CLIMBER_WINCH_MOTOR_LEFT,
      MotorType.kBrushless
    );
    m_motorLeft.configure(
      CLIMBER_WINCH.MOTOR_CONFIG_LEFT,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
    m_motorRight = new SparkMax(
      CAN_ID.CLIMBER_WINCH_MOTOR_RIGHT,
      MotorType.kBrushless
    );
    m_motorRight.configure(
      CLIMBER_WINCH.MOTOR_CONFIG_RIGHT,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void setSpeed(double percentOutput) {
    m_motorLeft.set(percentOutput);
    m_motorRight.set(percentOutput);
  }

  public void setSpeed(double leftOutput, double rightOutput) {
    m_motorLeft.set(leftOutput);
    m_motorRight.set(rightOutput);
  }

  public void setAxisSpeed(double speed) {
    speed *= CLIMBER_WINCH.AXIS_MAX_SPEED;
    m_motorLeft.set(speed);
    m_motorRight.set(speed);
  }

  public void stop() {
    m_motorLeft.stopMotor();
    m_motorRight.stopMotor();
  }
}
