// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class CAN_ID {

    public static final int ELEVATOR_LEFT_MOTOR = 23;
    public static final int ELEVATOR_RIGHT_MOTOR = 24;
  }

  public static class OPERATOR_CONSTANTS {

    public static final int kDriverControllerPort = 0;
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

  public static final class PHOTON {

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
  }
}
