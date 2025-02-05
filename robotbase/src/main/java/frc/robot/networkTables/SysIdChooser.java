package frc.robot.networkTables;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import java.util.HashMap;

public class SysIdChooser {

  private String[] m_autoNames;
  private SendableChooser<String> m_chooser;
  private SelectCommand<String> m_selectCommand;

  public SysIdChooser() {
    Command[] autoCommands = {
      Robot.swerve.sysIdDynamic(Direction.kForward).withName("Dynamic Forward"),
      Robot.swerve.sysIdDynamic(Direction.kReverse).withName("Dynamic Reverse"),
      Robot.swerve
        .sysIdQuasistatic(Direction.kForward)
        .withName("Quasistatic Forward"),
      Robot.swerve
        .sysIdQuasistatic(Direction.kReverse)
        .withName("Quasistatic Reverse"),
    };

    HashMap<String, Command> commandMap = new HashMap<String, Command>(
      autoCommands.length + 1
    );
    m_autoNames = new String[autoCommands.length + 1];

    for (int i = 0; i < autoCommands.length; i++) {
      commandMap.put(autoCommands[i].toString(), autoCommands[i]);
      m_autoNames[i + 1] = autoCommands[i].getName();
    }

    m_selectCommand = new SelectCommand<String>(commandMap, () ->
      m_chooser.getSelected()
    );

    m_chooser = new SendableChooser<String>();

    m_chooser.setDefaultOption("None", "None");
    for (String autoName : m_autoNames) {
      m_chooser.addOption(autoName, autoName);
    }
    SmartDashboard.putData("Auto chooser", m_chooser);
  }

  public Command getSelectedAutoCommand() {
    return m_selectCommand;
  }
}
