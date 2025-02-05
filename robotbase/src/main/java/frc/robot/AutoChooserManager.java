package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;

public class AutoChooserManager {

  // The map of named commands we use in choreo
  private static Map<String, Command> m_autoCommandsToBind = Map.of();

  // The map of auto routines that will show up on the auto command chooser.
  private static Map<String, AutoRoutine> m_autoRoutinesToBind = Map.of(
    "Cube test path",
    Robot.autos.cubeTestPath(),
    "Robot Relative Test",
    Robot.autos.robotRelativeTest()
  );

  private static AutoChooser m_autoChooser;

  public AutoChooserManager() {
    m_autoChooser = new AutoChooser();

    SendableBuilderImpl autoChooserBuilder = new SendableBuilderImpl();
    autoChooserBuilder.setTable(
      NetworkTableInstance.getDefault().getTable("SmartDashboard/Auto chooser")
    );

    m_autoChooser.initSendable(autoChooserBuilder);

    m_autoCommandsToBind.forEach((String name, Command command) -> {
      m_autoChooser.addCmd(name, () -> command);
    });
    m_autoRoutinesToBind.forEach((String name, AutoRoutine routine) -> {
      m_autoChooser.addRoutine(name, () -> routine);
    });

    SmartDashboard.putData("Auto chooser", m_autoChooser);
    SmartDashboard.putNumber("wait seconds", 0.0);
  }

  public Command getSelectedCommandScheduler() {
    return m_autoChooser.selectedCommandScheduler();
  }

  public Command getSelectedCommand() {
    return m_autoChooser.selectedCommand();
  }

  public String selectAuto(String autoToSelect) {
    if (autoToSelect == null) {
      return "Null string was given";
    }
    return m_autoChooser.select(autoToSelect);
  }
}
