package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ROBOT_CONFIGURATION.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

// these tell us the robot configuration, since its all calculations.
public class RobotConfigurationTests {

  @Test
  void MOITest() {
    System.out.println(
      "Mass Moment of Inertia - Simplified Mass Distribution : " +
      MOMENT_OF_INERTIA_SIMPLIFIED_DISTRIBUTION +
      " Kg / M^2"
    );
    assertTrue(MOMENT_OF_INERTIA_SIMPLIFIED_DISTRIBUTION > 0); // if its zero, somethings off. Not that it matters too much.
    assertTrue(MOMENT_OF_INERTIA_SIMPLIFIED_DISTRIBUTION < 10); // if its over 10, sanity check it. doesnt mean its wrong, just make sure it makes sense.
  }

  @Test
  void FullDimensionsTest() {
    System.out.println(
      "Bumper Thickness: " + BUMPER_THICKNESS.in(Inches) + "\""
    );
    System.out.println(
      "Frame Dimensions: " +
      FRAME_LENGTH.in(Inches) +
      "\" long " +
      FRAME_WIDTH.in(Inches) +
      "\" wide"
    );
    System.out.println(
      "Full Dimensions: " +
      FULL_LENGTH.in(Inches) +
      "\" long " +
      FULL_WIDTH.in(Inches) +
      "\" wide"
    );
    assertEquals(30, FULL_LENGTH.in(Inches), 2);
    assertEquals(30, FULL_WIDTH.in(Inches), 2);
  }

  @Test
  void BoundaryTest() {
    System.out.println("Robot Boundary: " + BOUNDARY.in(Inches) + "\"");
    assertEquals(20, BOUNDARY.in(Inches), 2); // boundary should be around 20 inches or so, depending on bumper thickness.
  }
}
