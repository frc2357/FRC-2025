// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import choreo.auto.AutoFactory;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.generated.TunerConstants;
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

  public static final class CAN_ID {

    public static final int PIGEON_ID = 5;
    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 11;
    public static final int FRONT_LEFT_STEER_MOTOR_ID = 12;
    public static final int FRONT_LEFT_ENCODER_ID = 19;

    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 13;
    public static final int FRONT_RIGHT_STEER_MOTOR_ID = 14;
    public static final int FRONT_RIGHT_ENCODER_ID = 20;

    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 15;
    public static final int BACK_LEFT_STEER_MOTOR_ID = 16;
    public static final int BACK_LEFT_ENCODER_ID = 21;

    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 17;
    public static final int BACK_RIGHT_STEER_MOTOR_ID = 18;
    public static final int BACK_RIGHT_ENCODER_ID = 22;

    public static final int ELEVATOR_LEFT_MOTOR = 23;
    public static final int ELEVATOR_RIGHT_MOTOR = 24;

    public static final int ALGAE_RUNNER_MOTOR = 25;
    public static final int LEFT_ALGAE_PIVOT_MOTOR = 26;
    public static final int RIGHT_ALGAE_PIVOT_MOTOR = 27;

    public static final int LATERATOR_MOTOR_LEFT = 28;
    public static final int LATERATOR_MOTOR_RIGHT = 29;

    public static final int CORAL_RUNNER_MOTOR = 30;
  }

  public final class DIGITAL_INPUT {

    public static final int LATERATOR_CENTER_HALL_EFFECT_SENSOR_ID = 0;
    public static final int CORAL_RUNNER_BEAM_BREAK_INTAKE_ID = 1;
    public static final int CORAL_RUNNER_BEAM_BREAK_OUTTAKE_ID = 2;
  }

  public static final class SWERVE {

    public static final AngularVelocity MAX_ANGULAR_VELOCITY =
      Units.RadiansPerSecond.of(Math.PI * 2);

    public static final double STATIC_FEEDFORWARD_METERS_PER_SECOND = 0.093545;

    public static final LinearAcceleration MAXIMUM_LINEAR_ACCELERATION =
      Units.MetersPerSecondPerSecond.of(4.5); //TODO: tune this
    public static final AngularAcceleration MAXIMUM_ANGULAR_ACCELERATION =
      Units.DegreesPerSecondPerSecond.of(120); //TODO: tune this
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
      Robot.swerve::getFieldRelativePose2d,
      Robot.swerve::setFieldRelativePose2d,
      Robot.swerve::followChoreoPath,
      true,
      Robot.swerve
    );
  }

  public static final class ELEVATOR {

    public static final SparkBaseConfig MOTOR_CONFIG_LEFT = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .openLoopRampRate(.25)
      .voltageCompensation(12);

    public static final SparkBaseConfig MOTOR_CONFIG_RIGHT =
      new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(.25)
        .voltageCompensation(12)
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

    public static final double MOTOR_P = 0;
    public static final double MOTOR_I = 0;
    public static final double MOTOR_D = 0;
    public static final double MOTOR_F = 0;

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      MOTOR_CONFIG_LEFT.closedLoop
        .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F)
        .outputRange(-1, 1);

    public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

    public static final double AXIS_MAX_SPEED = 0.1;

    public static final MAXMotionConfig MAX_MOTION_CONFIG_LEFT =
      CLOSED_LOOP_CONFIG_LEFT.maxMotion
        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
        .maxAcceleration(0)
        .maxVelocity(0);
  }

  public static final class CORAL_RUNNER {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .openLoopRampRate(.25)
      .voltageCompensation(12);

    public static final double MOTOR_P = 0;
    public static final double MOTOR_I = 0;
    public static final double MOTOR_D = 0;
    public static final double MOTOR_F = 0;

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG =
      MOTOR_CONFIG.closedLoop
        .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F)
        .outputRange(-1, 1);

    public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

    public static final double AXIS_MAX_SPEED = 0.1;

    public static final MAXMotionConfig MAX_MOTION_CONFIG =
      CLOSED_LOOP_CONFIG.maxMotion
        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
        .maxAcceleration(0)
        .maxVelocity(0);

    public static final double DEBOUNCE_TIME_SECONDS = 0.02;
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
      )
      .openLoopRampRate(RAMP_RATE);
  }

  public static final class ALGAE_PIVOT {

    public static final double AXIS_MAX_SPEED = 0.8;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final boolean MOTOR_INVERTED = false;

    public static final Current MOTOR_STALL_LIMIT = Units.Amps.of(50);
    public static final Current MOTOR_FREE_LIMIT = Units.Amps.of(50);

    public static final double RAMP_RATE = .25;

    public static final Angle MIN_ANGLE = Units.Degrees.of(0);
    public static final Angle MAX_ANGLE = Units.Degrees.of(90);

    public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0;

    public static final SparkBaseConfig LEFT_MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IDLE_MODE)
      .inverted(MOTOR_INVERTED)
      .smartCurrentLimit(
        (int) MOTOR_STALL_LIMIT.in(Units.Amps),
        (int) MOTOR_FREE_LIMIT.in(Units.Amps)
      )
      .openLoopRampRate(RAMP_RATE);

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

    public static final String FRONT_CAMERA_NAME = "shooter_camera";
    public static final Transform3d FRONT_CAMERA_TRANSFORM = new Transform3d(
      0,
      0,
      0,
      new Rotation3d(
        Units.Degrees.of(0),
        Units.Degrees.of(30),
        Units.Degrees.of(180)
      )
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
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE;

    // coeffiecients for pose trust from vision. Can be raised or lowered depending on how much we trust them.
    // yes, these are essentially magic numbers
    public static final double X_STD_DEV_COEFFIECIENT = 0.4;
    public static final double Y_STD_DEV_COEFFIECIENT = 0.4;

    // if were going faster than this, we wont accept any pose est.
    public static final LinearVelocity MAX_ACCEPTABLE_VELOCITY =
      Units.MetersPerSecond.of(3.5);

    // how close the estimated pose can get to the field border before we invalidate it
    public static final Distance FIELD_BORDER_MARGIN = Units.Meters.of(0.5);

    // how far off on the z axis the estimated pose can be before we invalidate it
    public static final Distance Z_MARGIN = Units.Feet.of(0.5);
  }

  public static final class FIELD_CONSTANTS {

    public static final Distance FIELD_LENGTH = Units.Feet.of(54).plus(
      Units.Inches.of(3)
    );
    public static final Distance FIELD_WIDTH = Units.Feet.of(26).plus(
      Units.Inches.of(3)
    );
  }

  public static class DRIVE_TO_POSE {

    public static final ProfiledPIDController AUTO_ALIGN_DRIVE_CONTROLLER =
      new ProfiledPIDController(
        8,
        0.0,
        0.0,
        new TrapezoidProfile.Constraints(2, 1)
      );

    public static final ProfiledPIDController AUTO_ALIGN_THETA_CONTROLLER =
      new ProfiledPIDController(
        6,
        0.0,
        0.0,
        new TrapezoidProfile.Constraints(2, 1)
      );

    public static final Distance X_TOLERANCE = Units.Inches.of(1);
    public static final Distance Y_TOLERANCE = Units.Inches.of(1);
    public static final Angle ROTATION_TOLERANCE = Units.Degrees.of(3);

    public static final double INTERPLOATION_PERCENT = 0.15;

    public static final Distance FINAL_APPROACH_DISTANCE = Units.Feet.of(3);

    public static final class COLLISION_AVOIDANCE {

      public static final int ATTEMPTS = 20;

      public static final double TWIST_X_METERS_DEFAULT = 0.05;
      public static final double TWIST_Y_METERS_DEFAULT = 0.05;
      public static final double TWIST_ROTO_RADIANS_DEFAULT = 0;

      /**
       * How far away we want to be from things that we could hit.
       */
      public static final Distance COLLISION_TOLERANCE = Units.Inches.of(3);

      /**
       * How close we want to get to the reef at any point in time. If were closer than this when traveling, a collision is likely.
       */
      public static final Distance REEF_BOUNDARY = FIELD.REEF.RADIUS.plus(
        ROBOT_CONFIGURATION.BOUNDARY
      ).plus(COLLISION_TOLERANCE);
    }

    public static final class WAYPOINTS {

      public static final Pose2d REEF_SIDE_A = new Pose2d(
        Meters.of(2.8930),
        Meters.of(3.9923),
        new Rotation2d(Degrees.of(0))
      );
      public static final Pose2d REEF_SIDE_B = new Pose2d(
        Meters.of(3.6986),
        Meters.of(2.6585),
        new Rotation2d(Radians.of(1.0191))
      );
      public static final Pose2d REEF_SIDE_C = new Pose2d(
        Meters.of(5.5078),
        Meters.of(2.2887),
        new Rotation2d(Radians.of(2.1253))
      );
      public static final Pose2d REEF_SIDE_D = new Pose2d(
        Meters.of(2.4567296504974365),
        Meters.of(4.0244574546813965),
        new Rotation2d(Degrees.of(180))
      );
      public static final Pose2d REEF_SIDE_E = new Pose2d(
        Meters.of(5.336165428161621),
        Meters.of(5.339362144470215),
        new Rotation2d(Radians.of(-2.1112160630631216))
      );
      public static final Pose2d REEF_SIDE_F = new Pose2d(
        Meters.of(3.7514467239379883),
        Meters.of(5.392186164855957),
        new Rotation2d(Radians.of(-1.0670657807739827))
      );
    }
  }

  public static final class CONTROLLER {

    public static final int DRIVE_CONTROLLER_PORT = 1;
    public static final double DRIVE_CONTROLLER_DEADBAND = 0.01;
    public static final int CODRIVER_CONTROLLER_PORT = 0;
    public static final double CODRIVE_CONTROLLER_DEADBAND = 0.025;
    public static final double SWERVE_TRANSLATIONAL_DEADBAND = 0.0;
    public static final double SWERVE_ROTATIONAL_DEADBAND = 0.0;
    public static final double DRIVE_RUMBLE_INTENSITY = .5;
    public static final double CODRIVE_RUMBLE_INTENSITY = .5;
    public static final double DRIVE_RUMBLE_SECONDS = 2;
    public static final double CODRIVE_RUMBLE_SECONDS = 2;
  }

  public static class FIELD { // pulled directly from Choreo or field drawings provided by FIRST

    public static class REEF {

      public static final Pose2d BRANCH_A = new Pose2d(
        Units.Meters.of(3.2332),
        Units.Meters.of(4.1914),
        new Rotation2d(Radians.of(0))
      );
      public static final Pose2d BRANCH_B = new Pose2d(
        Units.Meters.of(3.2332),
        Units.Meters.of(3.8564),
        new Rotation2d(Radians.of(0))
      );
      public static final Pose2d BRANCH_C = new Pose2d(
        Units.Meters.of(3.7160),
        Units.Meters.of(3.0202),
        new Rotation2d(Radians.of(1.0441))
      );
      public static final Pose2d BRANCH_D = new Pose2d(
        Units.Meters.of(4.0011),
        Units.Meters.of(2.8563),
        new Rotation2d(Radians.of(1.0441))
      );
      public static final Pose2d BRANCH_E = new Pose2d(
        Units.Meters.of(4.9734),
        Units.Meters.of(2.8552),
        new Rotation2d(Radians.of(2.0956))
      );
      public static final Pose2d BRANCH_F = new Pose2d(
        Units.Meters.of(5.2600),
        Units.Meters.of(3.0165),
        new Rotation2d(Radians.of(2.0956))
      );
      public static final Pose2d BRANCH_G = new Pose2d(
        Units.Meters.of(5.7408),
        Units.Meters.of(3.8570),
        new Rotation2d(Degrees.of(180))
      );
      public static final Pose2d BRANCH_H = new Pose2d(
        Units.Meters.of(5.7408),
        Units.Meters.of(4.1828),
        new Rotation2d(Degrees.of(180))
      );
      public static final Pose2d BRANCH_I = new Pose2d(
        Units.Meters.of(5.2650),
        Units.Meters.of(5.0293),
        new Rotation2d(Radians.of(-2.0970))
      );
      public static final Pose2d BRANCH_J = new Pose2d(
        Units.Meters.of(4.9792),
        Units.Meters.of(5.1939),
        new Rotation2d(Radians.of(-2.0970))
      );
      public static final Pose2d BRANCH_K = new Pose2d(
        Units.Meters.of(4.0037),
        Units.Meters.of(5.1982),
        new Rotation2d(Radians.of(-1.0505))
      );
      public static final Pose2d BRANCH_L = new Pose2d(
        Units.Meters.of(3.7203),
        Units.Meters.of(5.0299),
        new Rotation2d(Radians.of(-1.0505))
      );
      public static final Pose2d CENTER = new Pose2d(
        Units.Meters.of(4.4894),
        Units.Meters.of(4.0135),
        new Rotation2d(Degrees.of(0))
      );
      public static final Distance RADIUS = Units.Inches.of(75.506);
    }
  }

  /**
   * Class for numbers like the robots weight, its dimensions, bumper thickness, and anything else that should be written down about the robot.
   */
  public static class ROBOT_CONFIGURATION {

    public static final double WEIGHT_POUNDS = 51.5;

    public static final Distance FRAME_LENGTH = Units.Inches.of(26);
    public static final Distance FRAME_WIDTH = Units.Inches.of(26);

    public static final Distance BUMPER_THICKNESS = Units.Inches.of(3);

    public static final Distance FULL_LENGTH = FRAME_LENGTH.plus(
      BUMPER_THICKNESS.times(2)
    );
    public static final Distance FULL_WIDTH = FRAME_WIDTH.plus(
      BUMPER_THICKNESS.times(2)
    );

    /**
     * The distance that for any given object, if it is closer to the robot than this, it is hitting it, or will hit it when the robot turns.
     * Do not let anything get inside this.
     */
    public static final Distance BOUNDARY = Units.Inches.of(41.0121933 / 2);
  }
}
