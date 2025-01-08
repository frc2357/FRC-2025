package frc.robot.commands.auto;

import static frc.robot.Constants.CHOREO.AUTO_FACTORY;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.util.VariableWaitCommand;

public class Autos {

  public Autos() {}

  public Command resetOdometry(String trajectoryName) {
    return Commands.sequence(
      AUTO_FACTORY.resetOdometry(trajectoryName),
      new VariableWaitCommand(() -> SmartDashboard.getNumber("wait seconds", 0))
    );
  }

  public AutoRoutine cubeTestPath() {
    AutoRoutine routine = AUTO_FACTORY.newRoutine("CubeTestPath");

    AutoTrajectory cubeTestPathTraj = routine.trajectory("CubeTestPath");

    routine
      .active()
      .onTrue(
        Commands.sequence(resetOdometry("CubeTestPath"), cubeTestPathTraj.cmd())
      );

    return routine;
  }
}
