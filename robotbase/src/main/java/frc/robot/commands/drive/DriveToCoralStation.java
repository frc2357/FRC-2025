package frc.robot.commands.drive;

import static frc.robot.Constants.FIELD.CORAL_STATION.LOWER_STATION_DESIRED_SLOT;
import static frc.robot.Constants.FIELD.CORAL_STATION.UPPER_STATION_DESIRED_SLOT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Utility;

public class DriveToCoralStation extends DriveToPoseHandler {

  public enum StationToGoTo {
    BlueSide,
    RedSide,
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
  protected Pose2d getNewTarget(Pose2d currTarget, Pose2d currPose) {
    m_finalGoal = getDesiredTarget(currPose);
    return super.getNewTarget(currTarget, currPose);
  }

  private Pose2d getDesiredTarget(Pose2d currPose) {
    switch (m_desiredStation) {
      case RedSide:
        return LOWER_STATION_DESIRED_SLOT;
      case BlueSide:
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
