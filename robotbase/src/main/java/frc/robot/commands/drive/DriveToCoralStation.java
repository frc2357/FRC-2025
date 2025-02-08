package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.DRIVE_TO_POSE.*;
import static frc.robot.Constants.FIELD.CORAL_STATION.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;
import frc.robot.commands.drive.DriveToPoseHandler.RouteAroundReef;
import frc.robot.util.CollisionDetection;
import frc.robot.util.Utility;
import java.util.function.Function;

public class DriveToCoralStation extends DriveToPoseHandler {

  public enum StationToGoTo {
    Upper,
    Lower,
    Fastest,
  }

  private StationToGoTo m_desiredStation;

  public DriveToCoralStation(
    StationToGoTo desiredStation,
    RouteAroundReef routeAroundReef
  ) {
    this(desiredStation, routeAroundReef, null);
  }

  public DriveToCoralStation(
    StationToGoTo desiredStation,
    RouteAroundReef routeAroundReef,
    Command finalApproachCommand
  ) {
    super(routeAroundReef, finalApproachCommand);
    m_desiredStation = desiredStation != null
      ? desiredStation
      : StationToGoTo.Fastest;
  }

  @Override
  protected void extraNewTargetChecks(Pose2d currTarget, Pose2d currPose) {
    m_finalGoal = getDesiredTarget(currPose);
  }

  private Pose2d getDesiredTarget(Pose2d currPose) {
    switch (m_desiredStation) {
      case Lower:
        return LOWER_STATION_DESIRED_SLOT;
      case Upper:
        return UPPER_STATION_DESIRED_SLOT;
      case Fastest:
      default:
        double upperStationDist = Utility.findDistanceBetweenPoses(
          currPose,
          UPPER_STATION_DESIRED_SLOT
        );
        double lowerStationDist = Utility.findDistanceBetweenPoses(
          currPose,
          LOWER_STATION_DESIRED_SLOT
        );
        return upperStationDist <= lowerStationDist
          ? UPPER_STATION_DESIRED_SLOT
          : LOWER_STATION_DESIRED_SLOT;
    }
  }
}
