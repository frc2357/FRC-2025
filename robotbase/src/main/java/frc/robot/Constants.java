// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class CAN_ID {

    public static final int ELEVATOR_LEFT_MOTOR = 23;
    public static final int ELEVATOR_RIGHT_MOTOR = 24;
    public static final int ALGAE_RUNNER_MOTOR = 25;
    public static final int LEFT_ALGAE_PIVOT_MOTOR = 26;
    public static final int RIGHT_ALGAE_PIVOT_MOTOR = 27;
  }

  public static final class CHOREO {

    public static final PIDController X_CONTROLLER = new PIDController(5, 0, 0);
    public static final PIDController Y_CONTROLLER = new PIDController(5, 0, 0);
    public static final PIDController ROTATION_CONTROLLER = new PIDController(
      1,
      0,
      0
    );

    public static final AutoFactory AUTO_FACTORY = new AutoFactory(
      Robot.swerve::getPose2d,
      Robot.swerve::setPose2d,
      Robot.swerve::followChoreoPath,
      true,
      Robot.swerve
    );
  }

  public static final class ELEVATOR {

    public static final SparkBaseConfig MOTOR_CONFIG_LEFT = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    public static final SparkBaseConfig MOTOR_CONFIG_RIGHT =
      new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .follow(CAN_ID.ELEVATOR_LEFT_MOTOR, true);
    public static final double LEFT_MOTOR_P = 0;
    public static final double LEFT_MOTOR_I = 0;
    public static final double LEFT_MOTOR_D = 0;
    public static final double LEFT_MOTOR_F = 0;
    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      MOTOR_CONFIG_LEFT.closedLoop
        .pidf(LEFT_MOTOR_P, LEFT_MOTOR_I, LEFT_MOTOR_D, LEFT_MOTOR_F)
        .outputRange(-1, 1);
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
    public static final double AXIS_MAX_SPEED = 0.1;
    public static final Distance[] ELEVATOR_HEIGHT_SETPOINTS = {};
  }

  public static final class ALGAE_RUNNER {

    public static final double AXIS_MAX_SPEED = 0.8;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final boolean MOTOR_INVERTED = false;

    public static final Current MOTOR_STALL_LIMIT = Units.Amps.of(50);
    public static final Current MOTOR_FREE_LIMIT = Units.Amps.of(50);

    public static final double RAMP_RATE = .25;

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IDLE_MODE)
      .inverted(MOTOR_INVERTED)
      .smartCurrentLimit(
        (int) MOTOR_STALL_LIMIT.in(Units.Amps),
        (int) MOTOR_FREE_LIMIT.in(Units.Amps)
      );
  }

  public static final class ALGAE_PIVOT {

    public static final double AXIS_MAX_SPEED = 0.8;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final boolean MOTOR_INVERTED = false;

    public static final Current MOTOR_STALL_LIMIT = Units.Amps.of(50);
    public static final Current MOTOR_FREE_LIMIT = Units.Amps.of(50);

    public static final double RAMP_RATE = .25;

    public static final SparkBaseConfig LEFT_MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IDLE_MODE)
      .inverted(MOTOR_INVERTED)
      .smartCurrentLimit(
        (int) MOTOR_STALL_LIMIT.in(Units.Amps),
        (int) MOTOR_FREE_LIMIT.in(Units.Amps)
      );

    public static final SparkBaseConfig RIGHT_MOTOR_CONFIG =
      new SparkMaxConfig()
        .idleMode(IDLE_MODE)
        .follow(CAN_ID.LEFT_ALGAE_PIVOT_MOTOR);

    public static final double LEFT_MOTOR_P = 0;
    public static final double LEFT_MOTOR_I = 0;
    public static final double LEFT_MOTOR_D = 0;
    public static final double LEFT_MOTOR_F = 0;

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      LEFT_MOTOR_CONFIG.closedLoop
        .pidf(LEFT_MOTOR_P, LEFT_MOTOR_I, LEFT_MOTOR_D, LEFT_MOTOR_F)
        .outputRange(-1, 1);
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

  public static final class PHOTON_VISION {

    public static final String FRONT_CAMERA_NAME = "test";
    public static final Transform3d FRONT_CAMERA_TRANSFORM = new Transform3d(
      0,
      0,
      0,
      new Rotation3d(0, 0, 0)
    );

    public static final String LOST_CONNECTION_ERROR_MESSAGE =
      "**************LOST CONNECTION WITH ORANGE PI";
    public static final String CONNECTION_REGAINED_MESSAGE =
      "CONNECTION REGAINED WITH ORANGE PI*********";

    public static final Angle BEST_TARGET_PITCH_TOLERANCE = Units.Degrees.of(4);

    public static final Angle MAX_ANGLE = Units.Degrees.of(35);

    public static final double MAX_REPROJECTION_ERROR_PIXELS = 50; //TODO: tune this to a reasonable degree.
    public static final double MAX_AMBIGUITY_TOLERANCE = 4; //TODO: tune this until its reasonable.

    public static final boolean ACTIVATE_TURBO_SWITCH = false;

    public static final PoseStrategy PRIMARY_STRATEGY =
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    public static final PoseStrategy FALLBACK_STRATEGY =
      PoseStrategy.CLOSEST_TO_LAST_POSE;
  }
}
