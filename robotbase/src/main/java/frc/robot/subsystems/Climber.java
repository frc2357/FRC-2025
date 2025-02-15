package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.CLIMBER;
import frc.robot.Constants.ELEVATOR;

public class Climber extends SubsystemBase {

  private SparkMax m_motorOne;
  private SparkMax m_motorTwo;
  private SparkMax m_motorThree;

  public Climber() {
    m_motorOne = new SparkMax(CAN_ID.CLIMBER_MOTOR_ONE, MotorType.kBrushless);
    m_motorOne.configure(
      CLIMBER.MOTOR_CONFIG_ONE,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    m_motorTwo = new SparkMax(CAN_ID.CLIMBER_MOTOR_TWO, MotorType.kBrushless);
    m_motorTwo.configure(
      CLIMBER.MOTOR_CONFIG_TWO,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    m_motorThree = new SparkMax(
      CAN_ID.CLIMBER_MOTOR_THREE,
      MotorType.kBrushless
    );
    m_motorThree.configure(
      CLIMBER.MOTOR_CONFIG_THREE,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

  public void setSpeed(double percentOutput) {
    m_motorOne.set(percentOutput);
  }

  public void setAxisSpeed(double speed) {
    speed *= CLIMBER.AXIS_MAX_SPEED;
    m_motorOne.set(speed);
  }

  public void stop() {
    m_motorOne.stopMotor();
  }
}
