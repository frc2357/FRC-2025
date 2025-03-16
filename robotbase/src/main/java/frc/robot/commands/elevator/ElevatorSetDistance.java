package frc.robot.commands.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Robot;

public class ElevatorSetDistance extends Command {

  public static int running = 0;
  private Distance m_distance;

  public ElevatorSetDistance(Distance distance) {
    m_distance = distance;
    addRequirements(Robot.elevator);
  }

  public void initialize() {
    running++;
  }

  public void execute() {
    if (
      Robot.swerve
        .getAbsoluteTranslationalVelocity()
        .lte(Constants.SWERVE.ROBOT_NO_TIP_SPEED) ||
      Robot.elevator.isGoingDown()
    ) {
      Robot.elevator.setTargetDistance(m_distance);
    } else {
      Robot.elevator.setTargetDistance(Robot.elevator.getDistance());
    }
  }

  @Override
  public boolean isFinished() {
    return m_distance.isNear(
      Robot.elevator.getDistance(),
      ELEVATOR.SMART_MOTION_ALLOWED_ERROR_ROTATIONS
    );
  }

  @Override
  public void end(boolean interrupted) {
    running--;
  }
}
