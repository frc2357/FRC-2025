package frc.robot.controls;

import static frc.robot.Constants.FIELD.REEF.BRANCH_A;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.controls.controllers.CommandButtonboardController;
import frc.robot.controls.controllers.CommandButtonboardController.ReefSide;
import frc.robot.controls.controllers.CommandButtonboardController.ScoringDirection;
import frc.robot.controls.controllers.CommandButtonboardController.ScoringLevel;
import frc.robot.controls.util.RumbleInterface;

public class Buttonboard implements Sendable, RumbleInterface {

  private CommandButtonboardController m_controller;

  private ReefSide m_selectedReefSide = ReefSide.None;
  private ScoringLevel m_selectedScoringLevel = ScoringLevel.None;
  private ScoringDirection m_selectScoringDirection = ScoringDirection.None;

  public Buttonboard(CommandButtonboardController controller) {
    m_controller = controller;

    mapControls();
  }

  public ReefSide getSelectedReefSide() {
    return m_selectedReefSide;
  }

  public ScoringLevel getSelectedScoringLevel() {
    return m_selectedScoringLevel;
  }

  public ScoringDirection getSelectedScoringDirection() {
    return m_selectScoringDirection;
  }

  private void mapControls() {
    m_controller.a().onTrue(new SetReefSide(ReefSide.A));
    m_controller.b().onTrue(new SetReefSide(ReefSide.B));
    m_controller.c().onTrue(new SetReefSide(ReefSide.C));
    m_controller.d().onTrue(new SetReefSide(ReefSide.D));
    m_controller.e().onTrue(new SetReefSide(ReefSide.E));
    m_controller.f().onTrue(new SetReefSide(ReefSide.F));
    m_controller.noSide().onTrue(new SetReefSide(ReefSide.None));

    m_controller.L1().onTrue(new SetScoringLevel(ScoringLevel.L1));
    m_controller.L2().onTrue(new SetScoringLevel(ScoringLevel.L2));
    m_controller.L3().onTrue(new SetScoringLevel(ScoringLevel.L3));
    m_controller.L4().onTrue(new SetScoringLevel(ScoringLevel.L4));
    m_controller.noLevel().onTrue(new SetScoringLevel(ScoringLevel.None));

    m_controller.left().onTrue(new SetScoringDirection(ScoringDirection.Left));
    m_controller
      .right()
      .onTrue(new SetScoringDirection(ScoringDirection.Right));
    m_controller
      .noDirection()
      .onTrue(new SetScoringDirection(ScoringDirection.None));
  }

  private class SetReefSide extends InstantCommand {

    public SetReefSide(ReefSide side) {
      super(() -> {
        m_selectedReefSide = side;
      });
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }

  private class SetScoringLevel extends InstantCommand {

    public SetScoringLevel(ScoringLevel lvl) {
      super(() -> {
        m_selectedScoringLevel = lvl;
      });
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }

  private class SetScoringDirection extends InstantCommand {

    public SetScoringDirection(ScoringDirection dir) {
      super(() -> {
        m_selectScoringDirection = dir;
      });
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }

  public Pose2d getPoseFromGoal() {
    // ReefSide goal = Robot.buttonboard.getSelectedReefSide();
    // ScoringDirection scoringDirection =
    //   Robot.buttonboard.getSelectedScoringDirection();

    // switch (scoringDirection) {
    //   case Left:
    //     switch (goal) {
    //       case A:
    //         return REEF.BRANCH_A;
    //       case B:
    //         return REEF.BRANCH_C;
    //       case C:
    //         return REEF.BRANCH_E;
    //       case D:
    //         return REEF.BRANCH_G;
    //       case E:
    //         return REEF.BRANCH_I;
    //       case F:
    //         return REEF.BRANCH_K;
    //       default:
    //         return new Pose2d(-1, -1, new Rotation2d(Units.Degrees.of(-1)));
    //     }
    //   case Right:
    //     switch (goal) {
    //       case A:
    //         return REEF.BRANCH_B;
    //       case B:
    //         return REEF.BRANCH_D;
    //       case C:
    //         return REEF.BRANCH_F;
    //       case D:
    //         return REEF.BRANCH_H;
    //       case E:
    //         return REEF.BRANCH_J;
    //       case F:
    //         return REEF.BRANCH_L;
    //       default:
    //         return new Pose2d(-1, -1, new Rotation2d(Units.Degrees.of(-1)));
    //     }
    //   default:
    //     return new Pose2d(-1, -1, new Rotation2d(Units.Degrees.of(-1)));
    // }
    return BRANCH_A;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("HID");
    builder.addStringProperty(
      "Reef Side",
      () -> getSelectedReefSide().name(),
      null
    );
    builder.addStringProperty(
      "Scoring Level",
      () -> getSelectedScoringLevel().name(),
      null
    );
    builder.addStringProperty(
      "Scoring Direction",
      () -> getSelectedScoringDirection().name(),
      null
    );
  }

  @Override
  public void setRumble(double intensity) {
    m_controller.setRumble(RumbleType.kBothRumble, intensity);
  }
}
