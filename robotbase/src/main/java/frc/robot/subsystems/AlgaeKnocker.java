package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ALGAE_KNOCKER;
import frc.robot.Constants.CAN_ID;

public class AlgaeKnocker extends SubsystemBase {

  private SparkMax m_motor;

  public AlgaeKnocker() {
    m_motor = new SparkMax(CAN_ID.ALGAE_KNOCKER_MOTOR, MotorType.kBrushless);

    m_motor.configure(
      ALGAE_KNOCKER.MOTOR_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void setSpeed(double percentOutput) {
    m_motor.set(percentOutput);
  }

  public void setAxisSpeed(double axisSpeed) {
    axisSpeed *= ALGAE_KNOCKER.AXIS_MAX_SPEED;
    setSpeed(axisSpeed);
  }

  public void stop() {
    m_motor.stopMotor();
  }
}
