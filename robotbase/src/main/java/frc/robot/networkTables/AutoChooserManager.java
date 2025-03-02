package frc.robot.networkTables;

import choreo.auto.AutoChooser;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoBase;
import frc.robot.commands.auto.CB3BlueStation2Peice;
import frc.robot.commands.auto.TuningPathFinal;
import frc.robot.commands.auto.TuningPathRot;
import frc.robot.commands.auto.TuningPathX;
import frc.robot.commands.auto.TuningPathY;
import java.util.Map;

public class AutoChooserManager {

  // The map of named commands we use in choreo
  private Map<String, Command> m_autoCommandsToBind = Map.of();

  // The auto routines that will show up on the auto command chooser.
  private AutoBase[] m_autos = {
    new AutoBase("CubeTestPath"),
    new CB3BlueStation2Peice(),
    new TuningPathX(),
    new TuningPathY(),
    new TuningPathRot(),
    new TuningPathFinal(),
  };

  private AutoChooser m_autoChooser = new AutoChooser();

  public AutoChooserManager() {
    SendableBuilderImpl autoChooserBuilder = new SendableBuilderImpl();
    autoChooserBuilder.setTable(
      NetworkTableInstance.getDefault().getTable("SmartDashboard/Auto chooser")
    );

    m_autoChooser.initSendable(autoChooserBuilder);

    m_autoCommandsToBind.forEach((String name, Command command) -> {
      m_autoChooser.addCmd(name, () -> command);
    });

    for (AutoBase auto : m_autos) {
      m_autoChooser.addRoutine(auto.toString(), auto::getRoutine);
    }

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
