// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.util.CollisionDetection;
import frc.robot.util.SATCollisionDetector.SATVector;
import frc.robot.util.Utility;
import java.util.Optional;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
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

    public static final int LATERATOR_MOTOR = 28;
    public static final int CORAL_RUNNER_MOTOR = 29;

    public static final int CLIMBER_WINCH_MOTOR_LEFT = 30;
    public static final int CLIMBER_WINCH_MOTOR_RIGHT = 31;
    public static final int CLIMBER_PIVOT_MOTOR = 32;

    public static final int ALGAE_KNOCKER_MOTOR = 33;
  }

  public final class DIGITAL_INPUT {

    public static final int LATERATOR_CENTER_HALL_EFFECT_SENSOR_ID = 9;
    public static final int CORAL_RUNNER_BEAM_BREAK_OUTTAKE_ID = 8;
    public static final int CORAL_RUNNER_BEAM_BREAK_INTAKE_ID = 7;
  }

  public static final class SWERVE {

    public static final AngularVelocity MAX_ANGULAR_VELOCITY =
      Units.RadiansPerSecond.of((Math.PI * 2) / 1.5);

    public static final double STATIC_FEEDFORWARD_METERS_PER_SECOND = 0.093545;

    public static final Time TIME_TO_COAST = Units.Seconds.of(3);

    public static final double FACING_ANGLE_P = 0;
    public static final double FACING_ANGLE_I = 0;
    public static final double FACING_ANGLE_D = 0;

    public static final LinearVelocity ROBOT_NO_TIP_SPEED =
      Units.MetersPerSecond.of(.5);
  }

  public static final class CHOREO {

    public static final PIDController X_CONTROLLER = new PIDController(5, 0, 0);
    public static final PIDController Y_CONTROLLER = new PIDController(5, 0, 0);
    public static final PIDController ROTATION_CONTROLLER = new PIDController(
      8,
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

    public static final AutoFactory Y_FLIPPED_FACTORY = new AutoFactory(
      () -> Robot.swerve.flipYAxis(Robot.swerve.getFieldRelativePose2d()),
      (Pose2d pose) -> Robot.swerve.resetPose(Robot.swerve.flipYAxis(pose)),
      Robot.swerve::followChoreoPath,
      true,
      Robot.swerve
    );

    public static final double PREPOSE_SECONDS = 0.05;
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

    public static final double LEFT_MOTOR_P = 0;
    public static final double LEFT_MOTOR_I = 0;
    public static final double LEFT_MOTOR_D = 0;
    public static final double LEFT_MOTOR_VEL_F = 0.0003;
    public static final double LEFT_MOTOR_ARB_F = 0.15;

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      MOTOR_CONFIG_LEFT.closedLoop
        .pidf(LEFT_MOTOR_P, LEFT_MOTOR_I, LEFT_MOTOR_D, LEFT_MOTOR_VEL_F)
        .outputRange(-1, 1);
    public static final Angle SMART_MOTION_ALLOWED_ERROR_ROTATIONS =
      Units.Rotations.of(0.1);

    @SuppressWarnings("removal")
    public static final SmartMotionConfig SMART_MOTION_CONFIG_LEFT =
      CLOSED_LOOP_CONFIG_LEFT.smartMotion
        .allowedClosedLoopError(
          SMART_MOTION_ALLOWED_ERROR_ROTATIONS.in(Units.Rotations)
        )
        .maxAcceleration(10000)
        .maxVelocity(4600);

    public static final double GEAR_RATIO = (38.0 / 14.0) * 2.0;

    public static final Distance HTD5_PULLEY_PITCH = Units.Millimeters.of(5);
    public static final double OUTPUT_PULLEY_NUMBER_OF_TEETH = 28;
    public static final Distance OUTPUT_PULLEY_CIRCUMFERENCE =
      HTD5_PULLEY_PITCH.times(OUTPUT_PULLEY_NUMBER_OF_TEETH);

    public static final double AXIS_MAX_SPEED = 0.5;
    public static final double ZERO_SPEED = -0.1;

    public static final double ZERO_STALL_AMPS = 34; //TODO: tune this ASAP.

    public static final Time ZERO_TIME = Units.Seconds.of(0.2);

    public static final double HOLD_VOLTAGE = 0.5;

    public static final class SETPOINTS {

      public static final Distance HOME = Units.Inches.of(2);

      public static final Distance INTAKE_PREPOSE = Units.Inches.of(1.1);
      public static final Distance L1_PREPOSE = Units.Inches.of(1);
      public static final Distance L2_PREPOSE = Units.Inches.of(8.43);
      public static final Distance L3_PREPOSE = Units.Inches.of(24.189);
      public static final Distance L4_PREPOSE = Units.Inches.of(49.5);
      public static final Distance LOW_ALGAE = Units.Inches.of(0.5);
      public static final Distance HIGH_ALGAE = Units.Inches.of(13);
    }

    public static final Time FULL_EXTENSION_TIME = Units.Seconds.of(0.5);

    public static final double DEBOUNCE_TIME_SECONDS = 0.02;
  }

  public static final class LATERATOR {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .openLoopRampRate(.25)
      .voltageCompensation(12)
      .smartCurrentLimit(40, 40);

    public static final double MOTOR_P = 0.0002;
    public static final double MOTOR_I = 0;
    public static final double MOTOR_D = 0;
    public static final double MOTOR_F = 0;
    public static final double MOTOR_VEL_FF = 0.0004;

    // Set feedback sensor to alternate encoder
    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      MOTOR_CONFIG.closedLoop
        .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F)
        .velocityFF(MOTOR_VEL_FF)
        .outputRange(-1, 1);

    public static final Angle SMART_MOTION_ALLOWED_ERROR_ROTATIONS =
      Units.Rotations.of(0.05);

    public static final double AXIS_MAX_SPEED = 0.5;

    @SuppressWarnings("removal")
    public static final SmartMotionConfig MAX_MOTION_CONFIG_LEFT =
      CLOSED_LOOP_CONFIG_LEFT.smartMotion
        .allowedClosedLoopError(
          SMART_MOTION_ALLOWED_ERROR_ROTATIONS.in(Units.Rotations)
        )
        .maxAcceleration(10000)
        .maxVelocity(3500);

    public static final double GEAR_RATIO = 15;
    public static final Distance OUTPUT_PULLEY_PITCH_DIAMETER =
      Units.Millimeters.of(46.188);
    public static final Distance OUTPUT_PULLEY_CIRCUMFERENCE =
      OUTPUT_PULLEY_PITCH_DIAMETER.times(Math.PI);

    public static final class SETPOINTS {

      public static final Distance INTAKE_PREPOSE = Units.Inches.of(3);
      public static final Distance HOME = Units.Inches.of(1);
      public static final Distance MAX_SAFE_SCORING_EXTENSION = Units.Inches.of(
        -1
      );
      public static final Distance L1_PREPOSE = Units.Inches.of(-2);
      public static final Distance L2_PREPOSE = Units.Inches.of(-6.1);
      public static final Distance L3_PREPOSE = Units.Inches.of(-6.1);
      public static final Distance L4_PREPOSE = Units.Inches.of(-6.25);
      public static final Distance FULL_SCORING_EXTENSION = Units.Inches.of(
        -6.6
      );
    }

    public static final double STALL_WAIT_TIME = .1;

    public static final double DEBOUNCE_TIME_SECONDS = 0.02;

    public static final double NOMINAL_AMP_LIMIT = 30;

    public static final double ZERO_SPEED = -0.05;
  }

  public static final class CORAL_RUNNER {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(true)
      .openLoopRampRate(.25)
      .voltageCompensation(12)
      .smartCurrentLimit(40, 40);

    public static final double STALL_AMPS = 40;

    public static final double DEBOUNCE_TIME_SECONDS = 0.02;

    public static final double AXIS_MAX_SPEED = 0.5;
    public static final Dimensionless FAST_INTAKE_PERCENT = Units.Percent.of(
      0.32
    );
    public static final Dimensionless SLOW_INTAKE_PERCENT = Units.Percent.of(
      0.2
    );
    public static final Dimensionless BACK_OUT_PERCENT = Units.Percent.of(-0.2);
    public static final Dimensionless SCORING_PERCENT_L4 = Units.Percent.of(
      0.5
    );
    public static final Dimensionless SCORING_PERCENT_OTHER = Units.Percent.of(
      0.3
    );

    public static final double TELEOP_SCORING_WAIT_TIME = 0;
    public static final double AUTO_SCORING_WAIT_TIME = 0.5;

    public static final double BACKOUT_TIME_SECONDS = 0.3;
  }

  public static final class ALGAE_KNOCKER {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kCoast)
      .inverted(false)
      .smartCurrentLimit(20, 20)
      .openLoopRampRate(0.25);

    public static final double AXIS_MAX_SPEED = 0.25;
    public static final double ALGAE_KNOCK_SPEED = 0;
  }

  public static class CLIMBER_WINCH {

    public static final SparkBaseConfig MOTOR_CONFIG_LEFT = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60, 60)
      .inverted(false);
    public static final SparkBaseConfig MOTOR_CONFIG_RIGHT =
      new SparkMaxConfig().apply(MOTOR_CONFIG_LEFT).inverted(true);

    public static final double AXIS_MAX_SPEED = 0.8;
  }

  public static class CLIMBER_PIVOT {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60, 60)
      .inverted(true);

    public static final double AXIS_MAX_SPEED = .25;

    public static final double HOLD_AGAINST_WINCH_SPEED = -.02;
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

    public static final String LOST_CONNECTION_ERROR_MESSAGE =
      "**************LOST CONNECTION WITH ORANGE PI";
    public static final String CONNECTION_REGAINED_MESSAGE =
      "CONNECTION REGAINED WITH ORANGE PI*********";

    public static final Angle BEST_TARGET_PITCH_TOLERANCE = Units.Degrees.of(4);

    public static final Angle MAX_ANGLE = Units.Degrees.of(35);

    public static final boolean ACTIVATE_TURBO_SWITCH = false;

    public static final PoseStrategy PRIMARY_STRATEGY =
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    public static final PoseStrategy FALLBACK_STRATEGY =
      PoseStrategy.LOWEST_AMBIGUITY;

    public static final double PNP_HEADING_SCALE_FACTOR = 4; // no touchy.

    public static final Optional<ConstrainedSolvepnpParams> POSE_EST_PARAMS =
      Optional.of(
        new ConstrainedSolvepnpParams(false, PNP_HEADING_SCALE_FACTOR)
      );

    // coeffiecients for pose trust from vision. Can be raised or lowered depending on how much we trust them.
    public static final double X_STD_DEV_COEFFIECIENT = 0.8;
    public static final double Y_STD_DEV_COEFFIECIENT = 0.8;

    // if were going faster than these, we wont accept any pose est.
    public static final AngularVelocity MAX_ACCEPTABLE_ROTATIONAL_VELOCITY =
      Units.RadiansPerSecond.of(1);

    public static final LinearVelocity MAX_ACCEPTABLE_TRANSLATIONAL_VELOCITY =
      Units.MetersPerSecond.of(1.5);

    public static final Time INFO_VALID_TIME = Units.Seconds.of(0.4);

    public static final double MAGIC_VEL_CONF_ADDEND = 0.6;

    public static final double MAGIC_VEL_CONF_EXPONENT = 1.3;
    public static final Distance MAX_DIST_FROM_CURR_POSE = Units.Meters.of(
      0.75
    );

    public static final Distance MAX_DIST_BETWEEN_ESTIMATES = Units.Meters.of(
      0.5
    );

    public static final int MIN_ALLOWED_CUMMULATIVE_TARGETS = 1;

    public static final Time ESTIMATE_TIMEOUT = Units.Milliseconds.of(120);

    public static final Rotation2d HEADING_TOLERANCE = Rotation2d.fromDegrees(
      15
    );

    public static final class BACK_RIGHT_CAM {

      public static final String NAME = "backRightCam";
      // real transform
      // public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
      //   Units.Inches.of(4.624),
      //   Units.Inches.of(7.799),
      //   Units.Inches.of(22.055),
      //   new Rotation3d(
      //     Units.Degrees.of(0),
      //     Units.Degrees.of(-10),
      //     Units.Degrees.of(180)
      //   )
      // );
      // lying transform
      public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
        Units.Inches.of(8.824),
        Units.Inches.of(9),
        Units.Inches.of(22.055),
        new Rotation3d(
          Units.Degrees.of(0),
          Units.Degrees.of(-10),
          Units.Degrees.of(180)
        )
      );
    }

    public static final class BACK_LEFT_CAM {

      public static final String NAME = "backLeftCam";
      // true transform
      // public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
      //   Units.Inches.of(-6.516),
      //   Units.Inches.of(-5.028),
      //   Units.Inches.of(21.137),
      //   new Rotation3d(
      //     Units.Degrees.of(0),
      //     Units.Degrees.of(-10),
      //     Units.Degrees.of(180)
      //   )
      // );
      // lying transform (that makes it work way better)
      public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
        Units.Inches.of(10.15),
        Units.Inches.of(-7),
        Units.Inches.of(21.137),
        new Rotation3d(
          Units.Degrees.of(0),
          Units.Degrees.of(-10),
          Units.Degrees.of(180)
        )
      );
    }
  }

  public static final class FIELD_CONSTANTS {

    public static final AprilTagFields APRIL_TAG_FIELD =
      AprilTagFields.k2025ReefscapeAndyMark;

    public static final AprilTagFieldLayout HOME_FIELD_LAYOUT =
      Utility.makeHomeField();

    public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      // AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
      HOME_FIELD_LAYOUT;

    public static final Distance FIELD_LENGTH = Units.Meters.of(
      APRIL_TAG_LAYOUT.getFieldLength()
    );
    public static final Distance FIELD_WIDTH = Units.Meters.of(
      APRIL_TAG_LAYOUT.getFieldWidth()
    );

    // how close the estimated pose can get to the field border before we invalidate it
    public static final Distance FIELD_BORDER_MARGIN = Units.Inches.of(0.1);

    // how far off on the z axis the estimated pose can be before we invalidate it
    public static final Distance Z_MARGIN = Units.Feet.of(0.5);
  }

  public static class DRIVE_TO_POSE {

    public static final Constraints DRIVE_DEFAULT_CONSTRAINTS =
      new TrapezoidProfile.Constraints(30, 9);
    public static final Constraints DRIVE_FINAL_APPROACH_CONSTRAINTS =
      new TrapezoidProfile.Constraints(10, 5);

    public static final Constraints THETA_DEFAULT_CONSTRAINTS =
      new TrapezoidProfile.Constraints(15, 9);

    public static final ProfiledPIDController DRIVE_CONTROLLER =
      new ProfiledPIDController(8, 0.0, 0.0, DRIVE_DEFAULT_CONSTRAINTS);

    public static final ProfiledPIDController THETA_CONTROLLER =
      new ProfiledPIDController(6, 0.0, 0.0, THETA_DEFAULT_CONSTRAINTS);

    public static final Distance X_TOLERANCE = Units.Inches.of(0.1);
    public static final Distance Y_TOLERANCE = Units.Inches.of(0.1);
    public static final Angle ROTATION_TOLERANCE = Units.Degrees.of(2);

    public static final Pose2d FINAL_APPROACH_TOLERANCE_POSE = new Pose2d(
      X_TOLERANCE,
      Y_TOLERANCE,
      new Rotation2d(ROTATION_TOLERANCE)
    );

    public static final Pose2d WAYPOINT_APPROACH_TOLERANCE_POSE = new Pose2d(
      Units.Inches.of(3),
      Units.Inches.of(3),
      Rotation2d.fromDegrees(35)
    );

    public static final Distance FINAL_APPROACH_DISTANCE = Units.Feet.of(1);

    public static final Distance INTERPOLATION_DISTANCE = Units.Meters.of(0.2);

    public static final Rotation2d ROTATE_AROUND_REEF_ROTATION = new Rotation2d(
      Units.Rotations.of(0.08)
    );

    public static final double[] DEFAULT_INTERPOLATION_PERCENTAGES = {
      .1,
      .2,
      .3,
      .4,
      .5,
      .6,
      .7,
      .8,
    };

    public static final Distance IDEAL_DISTANCE_FROM_REEF =
      COLLISION_DETECTION.REEF_BOUNDARY.plus(Units.Feet.of(4));
  }

  public static final class COLLISION_DETECTION {

    public static final Distance COLLISION_TOLERANCE = Units.Inches.of(8);

    public static final Distance REEF_BOUNDARY = FIELD.REEF.DIAMETER.div(2)
      .plus(ROBOT_CONFIGURATION.BOUNDARY)
      .plus(COLLISION_TOLERANCE);

    public static final SATVector[] REEF_SAT_POLY =
      CollisionDetection.createReefPolygon();
  }

  public static final class CONTROLLER {

    public static final double SWERVE_TRANSLATIONAL_DEADBAND = 0.0;
    public static final double SWERVE_ROTATIONAL_DEADBAND = 0.0;

    public static final int DRIVE_CONTROLLER_PORT = 1;
    public static final int CODRIVER_CONTROLLER_PORT = 0;
    public static final int BUTTONBOARD_CONTROLLER_PORT = 2;

    public static final double DRIVE_CONTROLLER_DEADBAND = 0.01;
    public static final double CODRIVE_CONTROLLER_DEADBAND = 0.025;

    public static final double DRIVE_RUMBLE_INTENSITY = .5;
    public static final double CODRIVE_RUMBLE_INTENSITY = .5;
    public static final double BUTTONBOARD_RUMBLE_INTENSITY = 1;

    public static final double DRIVE_RUMBLE_SECONDS = 1;
    public static final double CODRIVE_RUMBLE_SECONDS = 1;
    public static final double BUTTONBOARD_RUMBLE_SECONDS = .05;
  }

  public static class FIELD { // pulled directly from Choreo or field drawings provided by FIRST

    public static class REEF {

      // the reef tags in order of what side of the reef they are on. do not sort these.
      public static final int[] BLUE_REEF_TAGS = { 18, 17, 22, 21, 20, 19 };
      public static final int[] RED_REEF_TAGS = { 7, 8, 9, 10, 11, 6 };

      public static final Pose2d BRANCH_A = new Pose2d(
        Units.Meters.of(3.48),
        Units.Meters.of(4.36),
        Rotation2d.k180deg
      );
      public static final Pose2d BRANCH_B = new Pose2d(
        Units.Meters.of(3.48),
        Units.Meters.of(4.01),
        Rotation2d.k180deg
      );
      public static final Pose2d BRANCH_C = new Pose2d(
        Units.Meters.of(3.82),
        Units.Meters.of(3.3),
        Rotation2d.fromDegrees(-117.85)
      );
      public static final Pose2d BRANCH_D = new Pose2d(
        Units.Meters.of(4.12),
        Units.Meters.of(3.2),
        Rotation2d.fromDegrees(-117.85)
      );
      public static final Pose2d BRANCH_E = new Pose2d(
        Units.Meters.of(5.04),
        Units.Meters.of(2.99),
        Rotation2d.fromRadians(-1.0600454389505496)
      );
      public static final Pose2d BRANCH_F = new Pose2d(
        Units.Meters.of(5.1060645),
        Units.Meters.of(3.173368),
        Rotation2d.fromRadians((2.0956364836439327 + 3.14))
      );
      public static final Pose2d BRANCH_G = new Pose2d(
        Units.Meters.of(5.7408),
        Units.Meters.of(3.8570),
        Rotation2d.kZero
      );
      public static final Pose2d BRANCH_H = new Pose2d(
        Units.Meters.of(5.7408),
        Units.Meters.of(4.1828),
        Rotation2d.kZero
      );
      public static final Pose2d BRANCH_I = new Pose2d(
        Units.Meters.of(5.310347859),
        Units.Meters.of(4.792431222),
        Rotation2d.fromRadians(1.044169055361146)
      );
      public static final Pose2d BRANCH_J = new Pose2d(
        Units.Meters.of(4.9792),
        Units.Meters.of(5.1939),
        Rotation2d.fromRadians(1.044169055361146)
      );
      public static final Pose2d BRANCH_K = new Pose2d(
        Units.Meters.of(4.0037),
        Units.Meters.of(5.1982),
        Rotation2d.fromRadians(2.0927415702150935)
      );
      public static final Pose2d BRANCH_L = new Pose2d(
        Units.Meters.of(3.7203),
        Units.Meters.of(5.0299),
        Rotation2d.fromRadians(2.0927415702150935)
      );
      public static final Pose2d CENTER = new Pose2d(
        Units.Meters.of(4.4894),
        Units.Meters.of(4.0135),
        Rotation2d.kZero
      );

      public static final Pose2d[] BRANCHES = {
        BRANCH_A,
        BRANCH_B,
        BRANCH_C,
        BRANCH_D,
        BRANCH_E,
        BRANCH_F,
        BRANCH_G,
        BRANCH_H,
        BRANCH_I,
        BRANCH_J,
        BRANCH_K,
        BRANCH_L,
      };
      public static final Pose2d BOTTOM_LEFT_CORNER = new Pose2d(
        3.6375527381896973,
        3.5441830158233643,
        Rotation2d.kZero
      );
      public static final Pose2d BOTTOM_CORNER = new Pose2d(
        4.485269546508789,
        3.054239273071289,
        Rotation2d.kZero
      );
      public static final Pose2d BOTTOM_RIGHT_CORNER = new Pose2d(
        5.330072402954102,
        3.548002004623413,
        Rotation2d.kZero
      );
      public static final Pose2d TOP_RIGHT_CORNER = new Pose2d(
        5.326759338378906,
        4.525322914123535,
        Rotation2d.kZero
      );
      public static final Pose2d TOP_CORNER = new Pose2d(
        4.48210334777832,
        5.012092113494873,
        Rotation2d.kZero
      );
      public static final Pose2d TOP_LEFT_CORNER = new Pose2d(
        3.635162591934204,
        4.51914644241333,
        Rotation2d.kZero
      );
      public static final Distance DIAMETER = Units.Inches.of(75.506);
    }

    public static class CORAL_STATION {

      public static final Pose2d LEFT_STATION_DESIRED_SLOT = new Pose2d(
        1.48,
        7.28,
        new Rotation2d(2.21946)
      );

      public static final Pose2d RIGHT_STATION_DESIRED_SLOT = new Pose2d(
        1.09824,
        0.94292,
        new Rotation2d(-2.21946)
      );
    }
  }

  /**
   * Class for numbers like the robots weight, its dimensions, bumper thickness, and anything else that should be written down about the robot.
   */
  public static class ROBOT_CONFIGURATION {

    public static final double WEIGHT_POUNDS = 135.3; // weight with battery and bumpers and all that

    public static final Distance FRAME_LENGTH = Units.Inches.of(26);
    public static final Distance FRAME_WIDTH = Units.Inches.of(26);

    public static final Distance BUMPER_THICKNESS = Units.Inches.of(3.125);

    public static final Distance FULL_LENGTH = FRAME_LENGTH.plus(
      BUMPER_THICKNESS.times(2)
    );
    public static final Distance FULL_WIDTH = FRAME_WIDTH.plus(
      BUMPER_THICKNESS.times(2)
    );

    public static final Transform2d FRONT_LEFT_CORNER_TRANSFORM =
      new Transform2d(
        FRAME_WIDTH.div(2),
        FRAME_LENGTH.div(2),
        Rotation2d.kZero
      );
    public static final Transform2d FRONT_RIGHT_CORNER_TRANSFORM =
      new Transform2d(
        FRAME_WIDTH.div(2),
        FRAME_LENGTH.div(2).unaryMinus(),
        Rotation2d.kZero
      );
    public static final Transform2d BACK_LEFT_CORNER_TRANSFORM =
      new Transform2d(
        FRAME_WIDTH.div(2).unaryMinus(),
        FRAME_LENGTH.div(2),
        Rotation2d.kZero
      );
    public static final Transform2d BACK_RIGHT_CORNER_TRANSFORM =
      new Transform2d(
        FRAME_WIDTH.div(2).unaryMinus(),
        FRAME_LENGTH.div(2).unaryMinus(),
        Rotation2d.kZero
      );

    /**
     * The distance that for any given object, if it is closer to the robot than this, it is hitting it, or will hit it when the robot turns.
     * Do not let anything get inside this.
     */
    public static final Distance BOUNDARY = Units.Inches.of(
      (Math.sqrt(
          Math.pow(FRAME_LENGTH.in(Units.Inches), 2) +
          Math.pow(FRAME_WIDTH.in(Units.Inches), 2)
        ) /
        2)
    );

    /**
     * The mass moment of intertia in Kg / M^2
     */
    public static final double MOMENT_OF_INERTIA_SIMPLIFIED_DISTRIBUTION =
      (1.0 / 12) *
      (WEIGHT_POUNDS / 2.205) */* pounds to kilograms conversion is / 2.205 */
      (Math.pow(FRAME_LENGTH.in(Units.Meters), 2) +
        Math.pow(FRAME_WIDTH.in(Units.Meters), 2));
  }
}
