package frc.robot.networkTables;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SignalLoggerManager implements Sendable {

  private boolean m_isLogging = false;

  public SignalLoggerManager() {
    SignalLogger.setPath("/media/sda1/logs");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Signal Logger");
    builder.addBooleanProperty(
      "Toggle Logger",
      () -> m_isLogging,
      value -> {
        if (value) {
          SignalLogger.start();
        } else {
          SignalLogger.stop();
        }
        m_isLogging = value;
      }
    );
  }
}
