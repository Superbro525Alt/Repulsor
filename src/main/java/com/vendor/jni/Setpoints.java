package com.vendor.jni;

import static com.vendor.jni.Constants.aprilTagLayout;
import static edu.wpi.first.units.Units.Meters;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Setpoints {
  public enum SetpointType {
    kHumanPlayer,
    kScore,
    kProcessor,
    kDeepCage,
    kShallowCage,
    kOther
  }

  public enum SetpointsReefscape {
    A(18, true, true),
    B(18, false, true),
    C(17, true, true),
    D(17, false, true),
    E(22, true, true),
    F(22, false, true),
    G(21, true, true),
    H(21, false, true),
    I(20, true, true),
    J(20, false, true),
    K(19, true, true),
    L(19, false, true),

    CLOSE(18, true, false),
    CLOSE_LEFT(19, true, false),
    CLOSE_RIGHT(17, true, false),
    FAR_RIGHT(22, true, false),
    FAR_LEFT(20, true, false),
    FAR(21, true, false),

    LEFT_HP(
        new Pose2d(1.148711085319519, 7.199769020080566, Rotation2d.fromDegrees(125.989 + 180))),
    RIGHT_HP(
        new Pose2d(
            0.9220133423805237,
            0.9964936375617981,
            Rotation2d.fromDegrees(125.989 + 180).unaryMinus()));

    private final Integer tagID;
    private final boolean isLeft;
    private final boolean isCoral;
    private final Pose2d staticPose;

    SetpointsReefscape(int tagID, boolean isLeft, boolean isCoral) {
      this.tagID = tagID;
      this.isLeft = isLeft;
      this.isCoral = isCoral;
      this.staticPose = null;
    }

    SetpointsReefscape(Pose2d staticPose) {
      this.staticPose = staticPose;
      this.tagID = null;
      this.isLeft = false;
      this.isCoral = false;
    }

    public Pose2d getPose(double robotX, double robotY, double coralOffset, double algaeOffset) {
      if (staticPose != null) return staticPose;

      var tagPose3dOpt = aprilTagLayout.getTagPose(tagID);

      if (tagPose3dOpt.isEmpty()) throw new RuntimeException("Missing tag: " + tagID);

      var tagPose3d = tagPose3dOpt.get();
      double offset = isCoral ? coralOffset : algaeOffset;

      Pose3d mappedPose = mapPose(tagPose3d, robotX, robotY);
      double baseAngle = mappedPose.getRotation().getAngle();
      double cos = Math.cos(baseAngle);
      double sin = Math.sin(baseAngle);
      double xOffset = offset * sin;
      double yOffset = offset * cos;

      Pose2d result;
      if (isLeft) {
        result =
            new Pose2d(
                mappedPose.getX() + xOffset,
                mappedPose.getY() - yOffset,
                mappedPose.getRotation().toRotation2d().plus(Rotation2d.kPi));
      } else {
        result =
            new Pose2d(
                mappedPose.getX() - xOffset,
                mappedPose.getY() + yOffset,
                mappedPose.getRotation().toRotation2d().plus(Rotation2d.kPi));
      }

      boolean isRed =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

      if (isRed) {
        return ChoreoAllianceFlipUtil.flip(result);
      }

      return result;
    }

    private static Pose3d mapPose(Pose3d pose, double robotX, double robotY) {
      double angle = pose.getRotation().getAngle();
      return new Pose3d(
          pose.getX() + Math.cos(angle) * robotX / 2,
          pose.getY() + Math.sin(angle) * robotY / 2,
          0.0,
          pose.getRotation());
    }

    public SetpointType type() {
      switch (this) {
        case A:
        case B:
        case C:
        case D:
        case E:
        case F:
        case G:
        case H:
        case I:
        case J:
        case K:
        case L:
          return SetpointType.kScore;

        case LEFT_HP:
        case RIGHT_HP:
          return SetpointType.kHumanPlayer;

        case CLOSE:
        case CLOSE_LEFT:
        case CLOSE_RIGHT:
        case FAR:
        case FAR_LEFT:
        case FAR_RIGHT:
          return SetpointType.kOther;

        default:
          throw new IllegalStateException("Unexpected value: " + this);
      }
    }
  }

  public enum HeightSetpoint {
    L1(Meters.of(0.46)),
    L2(Meters.of(0.81)),
    L3(Meters.of(1.21)),
    L4(Meters.of(1.83)),
    PROCESSOR(Meters.of(0.18)),
    DEEP_CAGE(Meters.of(0.75)),
    SHALLOW_CAGE(Meters.of(0.09)),
    CORAL_STATION(Meters.of(0.95)),
    NET(Meters.of(1.93)),
    NONE(null);

    private final Distance height;

    HeightSetpoint(Distance height) {
      this.height = height;
    }

    public Distance getHeight() {
      return height;
    }
  }

  public record RepulsorSetpoint(SetpointsReefscape point, HeightSetpoint h) {}
}
