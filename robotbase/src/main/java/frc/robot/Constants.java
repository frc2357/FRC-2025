// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.util.CollisionDetection;
import frc.robot.util.SATCollisionDetector.SATVector;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
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

    public static final int ALGAE_RUNNER_MOTOR = 25;
    public static final int LEFT_ALGAE_PIVOT_MOTOR = 26;
    public static final int RIGHT_ALGAE_PIVOT_MOTOR = 27;

    public static final int LATERATOR_MOTOR = 28;
    public static final int CORAL_RUNNER_MOTOR = 29;

    public static final int CLIMBER_MOTOR_ONE = 30;
    public static final int CLIMBER_MOTOR_TWO = 31;
    public static final int CLIMBER_MOTOR_THREE = 32;
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

    public static final Time TIME_TO_COAST = Units.Seconds.of(3);

    public static final double FACING_ANGLE_P = 0;
    public static final double FACING_ANGLE_I = 0;
    public static final double FACING_ANGLE_D = 0;
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
      .smartCurrentLimit(40, 40)
      .voltageCompensation(12);

    public static final SparkBaseConfig MOTOR_CONFIG_RIGHT =
      new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(.25)
        .voltageCompensation(12)
        .smartCurrentLimit(40, 40)
        .follow(CAN_ID.ELEVATOR_LEFT_MOTOR, true);

    public static final double LEFT_MOTOR_P = 0.008;
    public static final double LEFT_MOTOR_I = 0;
    public static final double LEFT_MOTOR_D = 0;
    public static final double LEFT_MOTOR_VEL_F = 0; // Should always be zero
    public static final double LEFT_MOTOR_ARB_F = 0.05;

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      MOTOR_CONFIG_LEFT.closedLoop
        .pidf(LEFT_MOTOR_P, LEFT_MOTOR_I, LEFT_MOTOR_D, LEFT_MOTOR_VEL_F)
        .outputRange(-1, 1);
    public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;
    public static final MAXMotionConfig MAX_MOTION_CONFIG_LEFT =
      CLOSED_LOOP_CONFIG_LEFT.maxMotion
        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
        .maxAcceleration(5000)
        .maxVelocity(4600);

    public static final int ENCODER_COUNTS_PER_REV = 8196;
    public static final double GEAR_RATIO = 3.2142857143;
    public static final Distance OUTPUT_PULLEY_DIAMETER = Units.Inches.of(
      2.256
    );

    public static final Distance HTD5_PULLEY_PITCH = Units.Millimeters.of(5);
    public static final double OUTPUT_PULLEY_NUMBER_OF_TEETH = 28;
    public static final Distance OUTPUT_PULLEY_CIRCUMFERENCE =
      HTD5_PULLEY_PITCH.times(OUTPUT_PULLEY_NUMBER_OF_TEETH);

    public static final double AXIS_MAX_SPEED = 0.5;

    public static final class SETPOINTS {

      public static final Distance HOME = Units.Feet.of(0); //TODO: Tune Setpoint

      public static final Distance INTAKE_PREPOSE = Units.Feet.of(0); //TODO: Tune Setpoint
      public static final Distance L1_PREPOSE = Units.Feet.of(0); //TODO: Tune Setpoint
      public static final Distance L2_PREPOSE = Units.Feet.of(0); //TODO: Tune Setpoint
      public static final Distance L3_PREPOSE = Units.Feet.of(0); //TODO: Tune Setpoint
      public static final Distance L4_PREPOSE = Units.Feet.of(0); //TODO: Tune Setpoint
    }
  }

  public static final class LATERATOR {

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .openLoopRampRate(.25)
      .voltageCompensation(12)
      .smartCurrentLimit(40, 40);

    public static final double MOTOR_P = 0;
    public static final double MOTOR_I = 0;
    public static final double MOTOR_D = 0;
    public static final double MOTOR_F = 0;

    // Set feedback sensor to alternate encoder
    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      MOTOR_CONFIG.closedLoop
        .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F)
        .outputRange(-1, 1);

    public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

    public static final double AXIS_MAX_SPEED = 0.5;

    public static final MAXMotionConfig MAX_MOTION_CONFIG_LEFT =
      CLOSED_LOOP_CONFIG_LEFT.maxMotion
        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
        .maxAcceleration(0)
        .maxVelocity(0);

    public static final double GEAR_RATIO = 5;
    public static final Distance OUTPUT_PULLEY_DIAMETER = Units.Millimeters.of(
      46.188
    );
    public static final Distance OUTPUT_PULLEY_CIRCUMFERENCE =
      OUTPUT_PULLEY_DIAMETER.times(Math.PI);

    public static final class SETPOINTS {

      public static final Distance HOME = Units.Feet.of(0); //TODO: Tune Setpoint

      public static final Distance INTAKE_PREPOSE = Units.Feet.of(0); //TODO: Tune Setpoint
      public static final Distance L1_PREPOSE = Units.Feet.of(0); //TODO: Tune Setpoint
      public static final Distance L2_PREPOSE = Units.Feet.of(0); //TODO: Tune Setpoint
      public static final Distance L3_PREPOSE = Units.Feet.of(0); //TODO: Tune Setpoint
      public static final Distance L4_PREPOSE = Units.Feet.of(0); //TODO: Tune Setpoint
    }
  }

  public static final class CORAL_RUNNER {

    // TODO: Tune speeds
    public static final Dimensionless FAST_INTAKE_PERCENT = Units.Percent.of(0);
    public static final Dimensionless SLOW_INTAKE_PERCENT = Units.Percent.of(0);
    public static final Dimensionless SCORING_PERCENT = Units.Percent.of(0);
    public static final double SCORING_WAIT_TIME = .5;

    public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .openLoopRampRate(.25)
      .voltageCompensation(12)
      .smartCurrentLimit(40, 40);

    public static final double AXIS_MAX_SPEED = 0.5;

    public static final double DEBOUNCE_TIME_SECONDS = 0.02;
  }

  public static final class ALGAE_RUNNER {

    public static final double AXIS_MAX_SPEED = 0.25;

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

    public static final double ALGAE_INTAKE_SPEED = 0;

    public static final double ALGAE_EJECTOR_SPEED = 0;
  }

  public static final class ALGAE_PIVOT {

    public static final double AXIS_MAX_SPEED = 0.25;

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
      .smartCurrentLimit(
        (int) MOTOR_STALL_LIMIT.in(Units.Amps),
        (int) MOTOR_FREE_LIMIT.in(Units.Amps)
      )
      .openLoopRampRate(RAMP_RATE);

    public static final SparkBaseConfig RIGHT_MOTOR_CONFIG =
      new SparkMaxConfig()
        .idleMode(IDLE_MODE)
        .openLoopRampRate(RAMP_RATE)
        .smartCurrentLimit(
          (int) MOTOR_STALL_LIMIT.in(Units.Amps),
          (int) MOTOR_FREE_LIMIT.in(Units.Amps)
        )
        .follow(CAN_ID.LEFT_ALGAE_PIVOT_MOTOR, true);

    public static final double LEFT_MOTOR_P = 0;
    public static final double LEFT_MOTOR_I = 0;
    public static final double LEFT_MOTOR_D = 0;
    public static final double LEFT_MOTOR_F = 0;

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      LEFT_MOTOR_CONFIG.closedLoop
        .pidf(LEFT_MOTOR_P, LEFT_MOTOR_I, LEFT_MOTOR_D, LEFT_MOTOR_F)
        .outputRange(-1, 1);
    //.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    public static final MAXMotionConfig MAX_MOTION_CONFIG_LEFT =
      CLOSED_LOOP_CONFIG_LEFT.maxMotion
        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
        .maxAcceleration(0)
        .maxVelocity(0);

    // public static final AbsoluteEncoderConfig ABSOLUTE_ENCODER_CONFIG_LEFT =
    //   LEFT_MOTOR_CONFIG.absoluteEncoder;

    public static final Angle ALGAE_INTAKE_ANGLE = Units.Degrees.of(0);
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

    public static final class FRONT_CAM {

      public static final String NAME = "frontCam";
      public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
        Units.Inches.of(7.951),
        Units.Inches.of(.624),
        Units.Inches.of(22.243),
        new Rotation3d(
          Units.Degrees.of(0),
          Units.Degrees.of(10),
          Units.Degrees.of(0)
        )
      );

      public static final Matrix<N3, N3> CAMERA_MATRIX = new Matrix<N3, N3>(
        new SimpleMatrix(
          new double[][] {
            { 734.557836120221, 0.0, 634.8211156347711 },
            { 0.0, 734.4550563158114, 504.1277599678814 },
            { 0.0, 0.0, 1.0 },
          }
        )
      );

      public static final Matrix<N8, N1> DIST_COEFFS = new Matrix<N8, N1>(
        new SimpleMatrix(
          new double[] {
            0.03900951112544531,
            -0.06701313537480716,
            -3.107885230659201E-4,
            -1.1708847785696965E-4,
            0.03884449647452857,
            -0.009013057952936134,
            0.012070659734909549,
            0.017173063089175628,
          }
        )
      );
    }

    public static final class BACK_CAM {

      public static final String NAME = "backCam";
      public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
        Units.Inches.of(-6.516),
        Units.Inches.of(-5.028),
        Units.Inches.of(21.137),
        new Rotation3d(
          Units.Degrees.of(0),
          Units.Degrees.of(-10),
          Units.Degrees.of(180)
        )
      );

      public static final Matrix<N3, N3> CAMERA_MATRIX = new Matrix<N3, N3>(
        new SimpleMatrix(
          new double[][] {
            { 731.8691015421067, 0.0, 647.4317928911091 },
            { 0.0, 732.0244798620424, 507.99253293961715 },
            { 0.0, 0.0, 1.0 },
          }
        )
      );

      public static final Matrix<N8, N1> DIST_COEEFS = new Matrix<N8, N1>(
        new SimpleMatrix(
          new double[] {
            0.038342519503234494,
            -0.0671687056289058,
            6.87360479856246E-5,
            4.596192020596837E-6,
            0.041912648539022886,
            -0.010420972593061864,
            0.014191022936485371,
            0.019150102220730315,
          }
        )
      );
    }

    public static final class RIGHT_CAM {

      public static final String NAME = "rightCam";
      public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
        Units.Inches.of(-8.887),
        Units.Inches.of(-3.001),
        Units.Inches.of(16.578),
        new Rotation3d(
          Units.Degrees.of(-10),
          Units.Degrees.of(0),
          Units.Degrees.of(90)
        )
      );

      public static final Matrix<N3, N3> CAMERA_MATRIX = new Matrix<N3, N3>(
        new SimpleMatrix(
          new double[][] {
            { 729.6608690553459, 0.0, 649.1575608574531 },
            { 0.0, 729.8029659622256, 529.4303396485709 },
            { 0.0, 0.0, 1.0 },
          }
        )
      );

      public static final Matrix<N8, N1> DIST_COEEFS = new Matrix<N8, N1>(
        new SimpleMatrix(
          new double[] {
            0.04116716900792142,
            -0.0683155445538715,
            -7.724921691039068E-5,
            -8.847337357162373E-5,
            0.03570970592766647,
            -0.007563612386900275,
            0.012708454838216305,
            0.01479948909867507,
          }
        )
      );
    }

    public static final class LEFT_CAM {

      public static final String NAME = "leftCam";
      public static final Transform3d ROBOT_TO_CAM_TRANSFORM = new Transform3d(
        Units.Inches.of(8.887),
        Units.Inches.of(-3.001),
        Units.Inches.of(16.579),
        new Rotation3d(
          Units.Degrees.of(10),
          Units.Degrees.of(0),
          Units.Degrees.of(270)
        )
      );

      public static final Matrix<N3, N3> CAMERA_MATRIX = new Matrix<N3, N3>(
        new SimpleMatrix(
          new double[][] {
            { 733.181868108721, 0.0, 652.9883355143077 },
            { 0.0, 732.9765658876138, 499.036857675139 },
            { 0.0, 0.0, 1.0 },
          }
        )
      );

      public static final Matrix<N8, N1> DIST_COEEFS = new Matrix<N8, N1>(
        new SimpleMatrix(
          new double[] {
            0.03808488979739667,
            -0.06777007253553732,
            -2.660953485058089E-4,
            2.389317050257177E-4,
            0.03942586698262671,
            -0.00896070145798233,
            0.011891179064366236,
            0.017476606427208132,
          }
        )
      );
    }

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
      PoseStrategy.CONSTRAINED_SOLVEPNP;
    public static final PoseStrategy FALLBACK_STRATEGY =
      PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;

    public static final Optional<ConstrainedSolvepnpParams> POSE_EST_PARAMS =
      Optional.of(new ConstrainedSolvepnpParams(false, 2)); // TODO: tune this throughout normal operation. This is for max to do.

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
        new TrapezoidProfile.Constraints(1.5, 1)
      );

    public static final ProfiledPIDController AUTO_ALIGN_THETA_CONTROLLER =
      new ProfiledPIDController(
        6,
        0.0,
        0.0,
        new TrapezoidProfile.Constraints(2, 1)
      );

    public static final Distance X_TOLERANCE = Units.Inches.of(3);
    public static final Distance Y_TOLERANCE = Units.Inches.of(3);
    public static final Angle ROTATION_TOLERANCE = Units.Degrees.of(6);

    public static final Distance FINAL_APPROACH_DISTANCE = Units.Feet.of(3);

    public static final Distance INTERPOLATION_DISTANCE = Units.Meters.of(0.35);

    public static final Rotation2d ROTATE_AROUND_REEF_ROTATIONS =
      new Rotation2d(Units.Rotations.of(0.05));

    public static final double[] DEFAULT_INTERPOLATION_PERCENTAGES = {
      .1,
      .2,
      .3,
      .4,
      .5,
      .6,
      .7,
      .8,
      .9,
    };

    /**
     * REEF_BOUNDARY + X distance away from the center of the reef
     */
    public static final Distance IDEAL_DISTANCE_FROM_REEF =
      COLLISION_DETECTION.REEF_BOUNDARY.plus(Units.Feet.of(1.5));

    /**
     * The slot number, starting at 1, from the alliance wall out, that we want to use. this can be changed on a per-match basis.
     */
    public static final int DESIRED_CORAL_STATION_SLOT_NUMBER = 2;
  }

  public static final class COLLISION_DETECTION {

    /**
     * How far away we want to be from things that we could hit.
     */
    public static final Distance COLLISION_TOLERANCE = Units.Inches.of(4);

    /**
     * How close we want to get to the reef at any point in time. If were closer than this when traveling, a collision is likely.
     */
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

    public static final double DRIVE_RUMBLE_SECONDS = 2;
    public static final double CODRIVE_RUMBLE_SECONDS = 2;
    public static final double BUTTONBOARD_RUMBLE_SECONDS = .05;
  }

  public static class FIELD { // pulled directly from Choreo or field drawings provided by FIRST

    public static class REEF {

      public static final Pose2d BRANCH_A = new Pose2d(
        Units.Meters.of(3.2332),
        Units.Meters.of(4.1914),
        Rotation2d.kZero
      );
      public static final Pose2d BRANCH_B = new Pose2d(
        Units.Meters.of(3.2332),
        Units.Meters.of(3.8564),
        Rotation2d.kZero
      );
      public static final Pose2d BRANCH_C = new Pose2d(
        Units.Meters.of(3.7160),
        Units.Meters.of(3.0202),
        new Rotation2d(Units.Radians.of(1.0441))
      );
      public static final Pose2d BRANCH_D = new Pose2d(
        Units.Meters.of(4.0011),
        Units.Meters.of(2.8563),
        new Rotation2d(Units.Radians.of(1.0441))
      );
      public static final Pose2d BRANCH_E = new Pose2d(
        Units.Meters.of(4.9734),
        Units.Meters.of(2.8552),
        new Rotation2d(Units.Radians.of(2.0956))
      );
      public static final Pose2d BRANCH_F = new Pose2d(
        Units.Meters.of(5.2600),
        Units.Meters.of(3.0165),
        new Rotation2d(Units.Radians.of(2.0956))
      );
      public static final Pose2d BRANCH_G = new Pose2d(
        Units.Meters.of(5.7408),
        Units.Meters.of(3.8570),
        Rotation2d.k180deg
      );
      public static final Pose2d BRANCH_H = new Pose2d(
        Units.Meters.of(5.7408),
        Units.Meters.of(4.1828),
        Rotation2d.k180deg
      );
      public static final Pose2d BRANCH_I = new Pose2d(
        Units.Meters.of(5.2650),
        Units.Meters.of(5.0293),
        new Rotation2d(Units.Radians.of(-2.0970))
      );
      public static final Pose2d BRANCH_J = new Pose2d(
        Units.Meters.of(4.9792),
        Units.Meters.of(5.1939),
        new Rotation2d(Units.Radians.of(-2.0970))
      );
      public static final Pose2d BRANCH_K = new Pose2d(
        Units.Meters.of(4.0037),
        Units.Meters.of(5.1982),
        new Rotation2d(Units.Radians.of(-1.0505))
      );
      public static final Pose2d BRANCH_L = new Pose2d(
        Units.Meters.of(3.7203),
        Units.Meters.of(5.0299),
        new Rotation2d(Units.Radians.of(-1.0505))
      );
      public static final Pose2d CENTER = new Pose2d(
        Units.Meters.of(4.4894),
        Units.Meters.of(4.0135),
        Rotation2d.kZero
      );
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

      public static final Pose2d UPPER_STATION_LEFTMOST_USABLE_SLOT = //TODO: tune this to field
        new Pose2d(
          0.5548880100250244,
          6.694406032562256,
          new Rotation2d(2.206778871255995)
        );
      public static final Transform2d UPPER_STATION_SLOT_TO_SLOT_TRANSFORM =
        new Transform2d(
          Units.Inches.of(5.65685),
          Units.Inches.of(5.65685),
          Rotation2d.kZero
        );

      public static final Pose2d UPPER_STATION_DESIRED_SLOT =
        UPPER_STATION_LEFTMOST_USABLE_SLOT.transformBy(
          UPPER_STATION_SLOT_TO_SLOT_TRANSFORM.times(
            DRIVE_TO_POSE.DESIRED_CORAL_STATION_SLOT_NUMBER
          )
        );

      public static final Pose2d LOWER_STATION_LEFTMOST_USABLE_SLOT = //TODO: tune this to field
        new Pose2d(
          0.5548880100250244,
          1.3386709690093994,
          new Rotation2d(-2.206778871255995)
        );
      public static final Transform2d LOWER_STATION_SLOT_TO_SLOT_TRANSFORM =
        new Transform2d(
          Units.Inches.of(5.65685),
          Units.Inches.of(-5.65685),
          Rotation2d.kZero
        );

      public static final Pose2d LOWER_STATION_DESIRED_SLOT =
        LOWER_STATION_LEFTMOST_USABLE_SLOT.transformBy(
          LOWER_STATION_SLOT_TO_SLOT_TRANSFORM.times(
            DRIVE_TO_POSE.DESIRED_CORAL_STATION_SLOT_NUMBER
          )
        );
    }
  }

  public static class CLIMBER {

    public static final IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    public static final Current STALL_LIMIT = Units.Amps.of(30);
    public static final Time RUN_DOWN_TIME = Units.Seconds.of(2);

    public static final double AXIS_MAX_SPEED = 0.25;

    public static final SparkBaseConfig MOTOR_CONFIG_ONE = new SparkMaxConfig()
      .idleMode(IDLE_MODE)
      .smartCurrentLimit((int) STALL_LIMIT.in(Units.Amps))
      .openLoopRampRate(1)
      .inverted(false);
    public static final SparkBaseConfig MOTOR_CONFIG_TWO = new SparkMaxConfig()
      .idleMode(IDLE_MODE)
      .smartCurrentLimit((int) STALL_LIMIT.in(Units.Amps))
      .openLoopRampRate(1)
      .follow(CAN_ID.CLIMBER_MOTOR_ONE);
    public static final SparkBaseConfig MOTOR_CONFIG_THREE =
      new SparkMaxConfig()
        .idleMode(IDLE_MODE)
        .smartCurrentLimit((int) STALL_LIMIT.in(Units.Amps))
        .openLoopRampRate(1)
        .follow(CAN_ID.CLIMBER_MOTOR_ONE);
  }

  /**
   * Class for numbers like the robots weight, its dimensions, bumper thickness, and anything else that should be written down about the robot.
   */
  public static class ROBOT_CONFIGURATION {

    public static final double WEIGHT_POUNDS = 105; // TODO: currently estimated. get weight once laterator is on, and then final weight.

    public static final Distance FRAME_LENGTH = Units.Inches.of(26);
    public static final Distance FRAME_WIDTH = Units.Inches.of(26);

    public static final Distance BUMPER_THICKNESS = Units.Inches.of(3);

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
      (WEIGHT_POUNDS / 2.205) */*pounds to kilograms conversion is / 2.205*/
      (Math.pow(FULL_LENGTH.in(Units.Meters), 2) +
        Math.pow(FULL_WIDTH.in(Units.Meters), 2));
  }
}
