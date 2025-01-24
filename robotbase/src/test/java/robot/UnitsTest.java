package robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

class UnitsTest {

  @Test
  void doesntWorkWhenClosed() {
    assertEquals(Double.NaN, Units.RPM.of(Double.NaN).baseUnitMagnitude(), 0.0);
  }
}
