package frc.robot.commands.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorSetDistance extends Command {

  private Distance m_distance;

  public ElevatorSetDistance(Distance distance) {
    m_distance = distance;
    addRequirements(Robot.elevator);
  }

  public void execute() {
    // if (
    //   Robot.swerve
    //     .getAbsoluteTranslationalVelocity()
    //     .lte(Constants.SWERVE.ROBOT_NO_TIP_SPEED) ||
    //   Robot.elevator.isGoingDown()
    // ) {
    Robot.elevator.setTargetDistance(m_distance);
    // } else {
    // Robot.elevator.setTargetDistance(Robot.elevator.getDistance());
    // }
  }

  @Override
  public boolean isFinished() {
    return Robot.elevator.isAtTarget();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.elevator.holdPosition();
  }
}
