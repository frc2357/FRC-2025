package frc.robot.commands.util;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import java.util.function.Consumer;

public class GetAlliance extends Command {

  private Consumer<Alliance> m_allianceConsumer;
  private Alliance m_fetchedAlliance;

  public GetAlliance(Consumer<Alliance> allianceConsumer) {
    m_allianceConsumer = allianceConsumer;
    m_fetchedAlliance = null;
  }

  @Override
  public void execute() {
    m_fetchedAlliance = DriverStation.getAlliance().orElse(null);
  }

  @Override
  public boolean isFinished() {
    return m_fetchedAlliance != null;
  }

  @Override
  public void end(boolean interrupted) {
    m_allianceConsumer.accept(m_fetchedAlliance);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
