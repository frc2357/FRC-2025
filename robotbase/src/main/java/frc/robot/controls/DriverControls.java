package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;
import frc.robot.commands.descoring.RemoveAlgaeHigh;
import frc.robot.commands.descoring.RemoveAlgaeLow;
import frc.robot.commands.drive.DriveRobotRelative;
import frc.robot.commands.drive.DriveToCoralStation;
import frc.robot.commands.drive.DriveToCoralStation.StationToGoTo;
import frc.robot.commands.drive.DriveToPoseHandler.RouteAroundReef;
import frc.robot.commands.drive.DriveToYawPitch;
import frc.robot.commands.drive.FlipPerspective;
import frc.robot.commands.intake.CoralIntake;
import frc.robot.commands.intake.CoralRetract;
import frc.robot.commands.scoring.CoralHome;
import frc.robot.commands.scoring.teleop.TeleopCoralScoreL2;
import frc.robot.commands.scoring.teleop.TeleopCoralScoreL3;
import frc.robot.commands.scoring.teleop.TeleopCoralScoreL4;
import frc.robot.commands.laterator.LateratorHome;
import frc.robot.commands.scoring.coral.CoralChooser;
import frc.robot.commands.scoring.coral.CoralHome;
import frc.robot.commands.scoring.coral.CoralScore;
import java.util.Optional;
import org.opencv.ml.Ml;

public class DriverControls {

  private CommandXboxController m_controller;

  private double m_deadband;

  private Trigger m_rightTrigger;
  private Trigger m_leftTrigger;

  public DriverControls(CommandXboxController controller, double deadband) {
    m_controller = controller;
    m_deadband = deadband;

    m_rightTrigger = m_controller.axisGreaterThan(Axis.kRightTrigger.value, 0);
    m_leftTrigger = m_controller.axisGreaterThan(Axis.kLeftTrigger.value, 0);

    mapControls();
  }

  public double getRightStickYAxis() {
    return m_controller.getRightY();
  }

  public void mapControls() {
    m_controller
      .start()
      .onTrue(
        new InstantCommand(() ->
          Robot.swerve.resetPose(
            REEF.BRANCH_A.plus(new Transform2d(-0.2, 0, Rotation2d.kZero))
          )
        )
      );

    // Scoring
    m_controller
      .leftBumper()
      .onTrue(new TeleopCoralScoreL4(m_rightTrigger).getCommand());
    m_controller
      .rightBumper()
      .onTrue(new TeleopCoralScoreL3(m_rightTrigger).getCommand());
    m_controller
      .x()
      .onTrue(new TeleopCoralScoreL2(m_rightTrigger).getCommand());

    // Intaking
    m_rightTrigger
      .and(() -> Robot.coralRunner.hasNoCoral())
      .toggleOnTrue(
        new CoralIntake().finallyDo(() -> new CoralRetract().schedule())
      );

    // Remove algae
    m_controller
      .a()
      .toggleOnTrue(
        new RemoveAlgaeLow().finallyDo(() -> new CoralHome().schedule())
      );
    m_controller
      .b()
      .toggleOnTrue(
        new RemoveAlgaeHigh().finallyDo(() -> new CoralHome().schedule())
      );

    // Other
    m_leftTrigger.onTrue(new CoralHome());
    m_controller.back().onTrue(new FlipPerspective());
    m_controller
      .start()
      .onTrue(new InstantCommand(() -> Robot.swerve.seedFieldCentric()));
    m_controller.y().whileTrue(new DriveRobotRelative());

    m_controller
      .y()
      .whileTrue(
        new DriveToCoralStation(StationToGoTo.LeftSide, RouteAroundReef.Fastest)
    m_controller.back().onTrue(new FlipPerspective());

    m_controller
      .povUp()
      .whileTrue(
        new DriveToYawPitch(
          () -> {
            Optional<Angle> yaw = Robot.backCam.getTargetYaw(18, 60);
            Optional<Angle> pitch = Robot.backCam.getTargetPitch(18, 60);
            if (yaw.isPresent() && pitch.isPresent()) {
              return Optional.of(
                new Translation2d(
                  yaw.get().in(Units.Degrees),
                  pitch.get().in(Units.Degrees)
                )
              );
            } else {
              return Optional.empty();
            }
          },
          () -> new Pose2d(-11, -2.3, new Rotation2d(0))
        )
      );
  }

  public double getX() {
    double value = -modifyAxis(m_controller.getLeftX());
    return Math.copySign(Math.pow(value, 2), value);
  }

  public double getY() {
    double value = -modifyAxis(m_controller.getLeftY());
    return Math.copySign(Math.pow(value, 2), value);
  }

  public double getRotation() {
    return -modifyAxis(m_controller.getRightX());
  }

  public double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public double modifyAxis(double value) {
    return deadband(value, m_deadband);
  }
}
