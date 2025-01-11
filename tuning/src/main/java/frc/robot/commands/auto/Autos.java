package frc.robot.commands.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.util.VariableWaitCommand;

import static frc.robot.Constants.CHOREO.AUTO_FACTORY;

public class Autos {
  public Autos(){

  }

  public Command resetOdometry(String trajectoryName){
    return Commands.sequence(
      AUTO_FACTORY.resetOdometry(trajectoryName),
      new VariableWaitCommand(() -> SmartDashboard.getNumber("wait seconds", 0))
    );

  }
  
  public AutoRoutine cubeTestPath(){
    AutoRoutine routine = AUTO_FACTORY.newRoutine("CubeTestPath");
    
    AutoTrajectory cubeTestPathTraj = routine.trajectory("CubeTestPath");
    
    routine.active().onTrue(
      Commands.sequence(
       resetOdometry("CubeTestPath"),
        cubeTestPathTraj.cmd()
      )
    );

    return routine;
  }
}
