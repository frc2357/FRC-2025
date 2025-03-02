package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public final class Constants {

  public static final class CAN_ID {

    public static final int ELEVATOR_LEFT_MOTOR = 23;
    public static final int ELEVATOR_RIGHT_MOTOR = 24;

    public static final int ALGAE_PIVOT_LEFT_MOTOR = 26;
    public static final int ALGAE_PIVOT_RIGHT_MOTOR = 27;

    public static final int LATERATOR_MOTOR = 28;
  }

  public static final class ELEVATOR {

    public static final SparkBaseConfig MOTOR_CONFIG_LEFT = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .openLoopRampRate(.25)
      .smartCurrentLimit(60, 40)
      .voltageCompensation(12);

    public static final SparkBaseConfig MOTOR_CONFIG_RIGHT =
      new SparkMaxConfig()
        .apply(MOTOR_CONFIG_LEFT)
        .follow(CAN_ID.ELEVATOR_LEFT_MOTOR, true);

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      MOTOR_CONFIG_LEFT.closedLoop.outputRange(-1, 1);

    public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

    public static final MAXMotionConfig MAX_MOTION_CONFIG_LEFT =
      CLOSED_LOOP_CONFIG_LEFT.maxMotion
        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
        .maxAcceleration(0)
        .maxVelocity(0);

    public static final double GEAR_RATIO = (38.0 / 14.0) * 2.0;

    public static final Distance HTD5_PULLEY_PITCH = Units.Millimeters.of(5);
    public static final double OUTPUT_PULLEY_NUMBER_OF_TEETH = 28;
    public static final Distance OUTPUT_PULLEY_CIRCUMFERENCE =
      HTD5_PULLEY_PITCH.times(OUTPUT_PULLEY_NUMBER_OF_TEETH);

    public static final double AXIS_MAX_SPEED = 0.5;
  }

  public static final class LATERATOR {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .openLoopRampRate(.25)
      .smartCurrentLimit(15)
      .voltageCompensation(12);

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG =
      MOTOR_CONFIG.closedLoop.outputRange(-1, 1);

    public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

    public static final double AXIS_MAX_SPEED = 0.75;

    public static final MAXMotionConfig MAX_MOTION_CONFIG =
      CLOSED_LOOP_CONFIG.maxMotion
        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
        .maxAcceleration(0)
        .maxVelocity(0);

    public static final double GEAR_RATIO = 15;
    public static final Distance OUTPUT_PULLEY_DIAMETER = Units.Millimeters.of(
      46.188
    );
    public static final Distance OUTPUT_PULLEY_CIRCUMFERENCE =
      OUTPUT_PULLEY_DIAMETER.times(Math.PI);
  }

  public static final class ALGAE_PIVOT {

    public static final SparkBaseConfig MOTOR_CONFIG_RIGHT =
      new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .openLoopRampRate(.25)
        .smartCurrentLimit(40, 20)
        .voltageCompensation(12);

    public static final SparkBaseConfig MOTOR_CONFIG_LEFT = new SparkMaxConfig()
      .apply(MOTOR_CONFIG_RIGHT)
      .follow(CAN_ID.ALGAE_PIVOT_RIGHT_MOTOR, true);

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_RIGHT =
      MOTOR_CONFIG_RIGHT.closedLoop
        .outputRange(-1, 1)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true);

    public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

    public static final MAXMotionConfig MAX_MOTION_CONFIG_RIGHT =
      CLOSED_LOOP_CONFIG_RIGHT.maxMotion
        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
        .maxAcceleration(0)
        .maxVelocity(0);

    public static final double AXIS_MAX_SPEED = 0.75;
  }

  public static final class CUSTOM_UNITS {

    // These units are ONLY for the output shaft on the neo. Any pulley will require
    // the addition of a gear ratio.
    public static final Distance NEO_SHAFT_CIRCUMFERENCE = Units.Millimeters.of(
      8 * Math.PI
    );
    public static final AngleUnit NEO_ENCODER_TICK = Units.derive(
      Units.Revolutions
    )
      .splitInto(42)
      .named("Neo Encoder Tick")
      .symbol("NET")
      .make();
  }
}
