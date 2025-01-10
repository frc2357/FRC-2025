package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.Autos;
import java.util.Map;

public class AutoChooserSetup {

  // The map of named commands we use in choreo
  private Map<String, Command> m_autoCommandsToBind = Map.of();

  // The map of auto routines that will show up on the auto command chooser.
  private Map<String, AutoRoutine> m_autoRoutinesToBind = Map.of(
    "Cube test path",
    Robot.autos.cubeTestPath()
  );

  public AutoChooser makeAutochooser() {
    AutoChooser autoChooser = new AutoChooser();

    SendableBuilderImpl autoChooserBuilder = new SendableBuilderImpl();
    autoChooserBuilder.setTable(
      NetworkTableInstance.getDefault().getTable("SmartDashboard/Auto chooser")
    );

    autoChooser.initSendable(autoChooserBuilder);

    m_autoCommandsToBind.forEach((String name, Command command) -> {
      autoChooser.addCmd(name, () -> command);
    });
    m_autoRoutinesToBind.forEach((String name, AutoRoutine routine) -> {
      autoChooser.addRoutine(name, () -> routine);
    });

    return autoChooser;
  }
}
