package frc.robot;

import static frc.robot.Constants.FIELD.REEF.BLUE_REEF_TAGS;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.CameraManager;
import java.util.List;
import org.junit.jupiter.api.Test;

public class PoseEstTests extends CameraManager {

  CameraManager m_camManager;

  // change this to true to print out a copy and pasteable AprilTagFieldLayout of the measured out home field tags.
  final boolean m_printOutHomeFieldCode = false;

  @Test
  void fieldMapTest() {
    if (!m_printOutHomeFieldCode) {
      return;
    }
    double correctDistFromLeftInches = 14.25;
    double[] tagDists = { 16.125, 14.25, 14.125, 16.4375, 16.375, 16.375 };
    var normalLayout = AprilTagFieldLayout.loadField(
      AprilTagFields.k2025ReefscapeAndyMark
    );
    var poses = normalLayout.getTags();
    System.out.println("NORMAL POSES --------------");
    printBlueReefTags(poses);
    System.out.println("------\nCORRECTING POSES\n------");
    for (int i = 0; i < BLUE_REEF_TAGS.length; i++) {
      int tag = BLUE_REEF_TAGS[i];
      var tagPose = poses.get(tag - 1).pose;
      var correctionTransform = new Transform2d(
        Units.Inches.zero(),
        Units.Inches.of(correctDistFromLeftInches - tagDists[i]),
        Rotation2d.kZero
      );
      System.out.println(
        "TAG ID: " + tag + " | TRANSFORM: " + correctionTransform
      );
      var correctedTagPose = tagPose.transformBy(
        new Transform3d(correctionTransform)
      );
      poses.set(tag - 1, new AprilTag(tag, correctedTagPose));
    }
    System.out.println(
      "---------------\nCORRECTED POSES ----------------------*"
    );
    printBlueReefTags(poses);
    printCopyableFieldLayoutCode(poses);
  }

  void printBlueReefTags(List<AprilTag> tagList) {
    for (int id : BLUE_REEF_TAGS) {
      var tag = tagList.get(id - 1);
      var tagPose = tag.pose.toPose2d();
      System.out.println(
        "TAG " +
        tag.ID +
        " | pose: x: " +
        tagPose.getX() +
        " | y: " +
        tagPose.getY()
      );
    }
  }

  void printCopyableFieldLayoutCode(List<AprilTag> tagList) {
    System.out.print("\nnew AprilTagFieldLayout(List.of(");
    for (AprilTag tag : tagList) {
      var tagPose = tag.pose;
      System.out.print(
        "new AprilTag(" +
        tag.ID +
        ", new Pose3d(" +
        tagPose.getX() +
        ", " +
        tagPose.getY() +
        ", " +
        tagPose.getZ() +
        ", new Rotation3d( " +
        tagPose.getRotation().getX() +
        ", " +
        tagPose.getRotation().getY() +
        ", " +
        tagPose.getRotation().getZ() +
        "))),"
      );
    }
    System.out.println("));");
  }
}
