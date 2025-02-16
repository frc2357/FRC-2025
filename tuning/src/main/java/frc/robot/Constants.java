package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class Constants {

  public static final class CAN_ID {

    public static final int ELEVATOR_LEFT_MOTOR = 23;
    public static final int ELEVATOR_RIGHT_MOTOR = 24;

    public static final int LATERATOR_MOTOR_LEFT = 28;
    public static final int LATERATOR_MOTOR_RIGHT = 29;

    public static final int CORAL_RUNNER_MOTOR = 30;
    public static final int LEFT_ALGAE_PIVOT_MOTOR = 26;
    public static final int RIGHT_ALGAE_PIVOT_MOTOR = 27;
  }

  public static final class ELEVATOR {

    public static final SparkBaseConfig MOTOR_CONFIG_LEFT = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .openLoopRampRate(.25)
      .smartCurrentLimit(40, 40)
      .voltageCompensation(12);

    public static final SparkBaseConfig MOTOR_CONFIG_RIGHT =
      new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(.25)
        .voltageCompensation(12)
        .smartCurrentLimit(40, 40)
        .follow(CAN_ID.ELEVATOR_LEFT_MOTOR, true);

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      MOTOR_CONFIG_LEFT.closedLoop.outputRange(-1, 1);

    public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

    public static final MAXMotionConfig MAX_MOTION_CONFIG_LEFT =
      CLOSED_LOOP_CONFIG_LEFT.maxMotion
        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
        .maxAcceleration(0)
        .maxVelocity(0);

    public static final int ENCODER_COUNTS_PER_REV = 8196;

    public static final double GEAR_RATIO = 50 / 14;
    public static final Distance MOTOR_PULLEY_PITCH_DIAMETER = Units.Inches.of(
      2.256
    );

    public static final double AXIS_MAX_SPEED = 0.25;
  }

  public static final class CORAL_RUNNER {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .openLoopRampRate(.25)
      .voltageCompensation(12);

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG =
      MOTOR_CONFIG.closedLoop.outputRange(-1, 1);

    public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

    public static final double AXIS_MAX_SPEED = 0.1;

    public static final MAXMotionConfig MAX_MOTION_CONFIG =
      CLOSED_LOOP_CONFIG.maxMotion
        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
        .maxAcceleration(0)
        .maxVelocity(0);

    public static final double DEBOUNCE_TIME_SECONDS = 0.02;
  }

  public static final class LATERATOR {

    public static final SparkBaseConfig MOTOR_CONFIG_LEFT = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .openLoopRampRate(.25)
      .voltageCompensation(12);

    public static final SparkBaseConfig MOTOR_CONFIG_RIGHT =
      new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .openLoopRampRate(.25)
        .voltageCompensation(12)
        .follow(CAN_ID.LATERATOR_MOTOR_LEFT);

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      MOTOR_CONFIG_LEFT.closedLoop.outputRange(-1, 1);

    public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

    public static final double AXIS_MAX_SPEED = 0.1;

    public static final MAXMotionConfig MAX_MOTION_CONFIG_LEFT =
      CLOSED_LOOP_CONFIG_LEFT.maxMotion
        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
        .maxAcceleration(0)
        .maxVelocity(0);
  }

  public static final class ALGAE_PIVOT {

    public static final double AXIS_MAX_SPEED = 0.8;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final boolean MOTOR_INVERTED = false;

    public static final Angle MIN_ANGLE = Units.Degrees.of(0);
    public static final Angle MAX_ANGLE = Units.Degrees.of(90);

    public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0;

    public static final SparkBaseConfig LEFT_MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IDLE_MODE)
      .inverted(MOTOR_INVERTED);

    public static final SparkBaseConfig RIGHT_MOTOR_CONFIG =
      new SparkMaxConfig()
        .idleMode(IDLE_MODE)
        .follow(CAN_ID.LEFT_ALGAE_PIVOT_MOTOR);

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      LEFT_MOTOR_CONFIG.closedLoop.outputRange(-1, 1);
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
