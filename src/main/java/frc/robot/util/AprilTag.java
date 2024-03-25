package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class AprilTag {

  public static final double[] tagHeights = {
    0, 1.36, 1.36, 1.45, 1.45, 1.36, 1.36, 1.45, 1.45, 1.36, 1.36, 1.32, 1.32, 1.32, 1.32, 1.32,
    1.32
  };

  public static final double speakerHeight = Units.inchesToMeters(80.5);
  public static final double ampHeight = Units.inchesToMeters(38);
  public static final double sourceHeight = Units.inchesToMeters(0);
  public static final double stageHeight = Units.inchesToMeters(0);

  public static final Pose3d speakerRed =
      new Pose3d(16.58, 5.55, speakerHeight, new Rotation3d(0, 0, Math.toRadians(180)));
  public static final Pose3d ampRed =
      new Pose3d(14.70, 8.20, ampHeight, new Rotation3d(0, 0, Math.toRadians(-90)));
  public static final Pose3d sourceRed =
      new Pose3d(0.91, 0.565, sourceHeight, new Rotation3d(0, 0, Math.toRadians(60)));
  public static final Pose3d[] nearestStageRed = {
    new Pose3d(11.90, 3.71, stageHeight, new Rotation3d(0, 0, Math.toRadians(-60))),
    new Pose3d(11.90, 4.50, stageHeight, new Rotation3d(0, 0, Math.toRadians(60))),
    new Pose3d(11.22, 4.11, stageHeight, new Rotation3d(0, 0, Math.toRadians(180)))
  };

  public static final Pose3d speakerBlue =
      new Pose3d(-0.04, 5.55, speakerHeight, new Rotation3d(0, 0, Math.toRadians(0)));
  public static final Pose3d ampBlue =
      new Pose3d(1.84, 8.20, ampHeight, new Rotation3d(0, 0, Math.toRadians(-90)));
  public static final Pose3d sourceBlue =
      new Pose3d(15.635, 0.565, sourceHeight, new Rotation3d(0, 0, Math.toRadians(120)));
  public static final Pose3d[] nearestStageBlue = {
    new Pose3d(5.32, 4.11, stageHeight, new Rotation3d(0, 0, Math.toRadians(0))),
    new Pose3d(4.64, 4.50, stageHeight, new Rotation3d(0, 0, Math.toRadians(120))),
    new Pose3d(4.64, 3.71, stageHeight, new Rotation3d(0, 0, Math.toRadians(-120)))
  };

  public static final Pose3d[] arrTranslations = {
    null,
    sourceBlue,
    sourceBlue,
    speakerRed,
    speakerRed,
    ampRed,
    ampBlue,
    speakerBlue,
    speakerBlue,
    sourceRed,
    sourceRed,
    nearestStageRed[0],
    nearestStageRed[1],
    nearestStageRed[2],
    nearestStageBlue[0],
    nearestStageBlue[1],
    nearestStageBlue[2]
  };

  public static final Pose3d[] tagPoses = {
    null,
    new Pose3d(15.08, 0.25, tagHeights[1], new Rotation3d(0, 0, Math.toRadians(120))), // Blue source r
    new Pose3d(16.19, 0.88, tagHeights[2], new Rotation3d(0, 0, Math.toRadians(120))), // Blue source l
    new Pose3d(16.58, 4.98, tagHeights[3], new Rotation3d(0, 0, Math.toRadians(180))), // 
    new Pose3d(16.58, 5.55, tagHeights[4], new Rotation3d(0, 0, Math.toRadians(180))),
    new Pose3d(14.70, 8.20, tagHeights[5], new Rotation3d(0, 0, Math.toRadians(-90))),
    new Pose3d(1.84, 8.20, tagHeights[6], new Rotation3d(0, 0, Math.toRadians(-90))),
    new Pose3d(-0.04, 5.55, tagHeights[7], new Rotation3d(0, 0, Math.toRadians(0))),
    new Pose3d(-0.04, 4.98, tagHeights[8], new Rotation3d(0, 0, Math.toRadians(0))),
    new Pose3d(0.36, 0.88, tagHeights[9], new Rotation3d(0, 0, Math.toRadians(60))),
    new Pose3d(1.46, 0.25, tagHeights[10], new Rotation3d(0, 0, Math.toRadians(60))),
    new Pose3d(11.90, 3.71, tagHeights[11], new Rotation3d(0, 0, Math.toRadians(-60))),
    new Pose3d(11.90, 4.50, tagHeights[12], new Rotation3d(0, 0, Math.toRadians(60))),
    new Pose3d(11.22, 4.11, tagHeights[13], new Rotation3d(0, 0, Math.toRadians(180))),
    new Pose3d(5.32, 4.11, tagHeights[14], new Rotation3d(0, 0, Math.toRadians(0))),
    new Pose3d(4.64, 4.50, tagHeights[15], new Rotation3d(0, 0, Math.toRadians(120))),
    new Pose3d(4.64, 3.71, tagHeights[16], new Rotation3d(0, 0, Math.toRadians(-120)))
  };

  public static Pose2d getTagPose(int tagId) {
    return tagPoses[tagId].toPose2d();
  }
}
