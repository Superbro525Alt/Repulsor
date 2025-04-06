package com.vendor.jni;

import com.vendor.jni.FieldPlanner.Obstacle;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public class ExtraPathing {
  public enum NavigationType {
    kMoveObstacles
  }

  public static boolean isClearPath(
      Translation2d current,
      Translation2d goal,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters) {
    double stepSize = 0.05;
    double pathLength = current.getDistance(goal);
    int samples = Math.max(2, (int) (pathLength / stepSize));

    Translation2d direction = goal.minus(current);
    Rotation2d heading = direction.getAngle();

    double halfLength = robotLengthMeters / 2.0;
    double halfWidth = robotWidthMeters / 2.0;

    for (int i = 1; i <= samples; i++) {
      double t = i / (double) samples;
      Translation2d center = current.interpolate(goal, t);

      Translation2d[] corners = {
        new Translation2d(-halfLength, -halfWidth),
        new Translation2d(halfLength, -halfWidth),
        new Translation2d(halfLength, halfWidth),
        new Translation2d(-halfLength, halfWidth)
      };

      for (int j = 0; j < 4; j++) {
        corners[j] = corners[j].rotateBy(heading).plus(center);
      }

      for (Obstacle obs : obstacles) {
        if (obs.intersectsRectangle(corners)) {
          return false;
        }
      }
    }

    return true;
  }

  /** A fallback plan that computes a new target Pose2d to use when the original path is blocked. */
  public abstract class FallbackPlan {
    /**
     * Calculates a new fallback target pose when the original target is unreachable.
     *
     * @param originalTarget The original target pose.
     * @param currentPose The current pose of the robot.
     * @return A new Pose2d representing the fallback target.
     */
    public abstract Pose2d calculate(Pose2d originalTarget, Pose2d currentPose);

    /**
     * Calculates a new fallback target pose using a Translation2d as the original target.
     *
     * @param originalTarget The original target location as a Translation2d.
     * @param currentPose The current pose of the robot.
     * @return A new Pose2d representing the fallback target.
     */
    public Pose2d calculate(Translation2d originalTarget, Pose2d currentPose) {
      return calculate(
          new Pose2d(originalTarget.getX(), originalTarget.getY(), new Rotation2d()), currentPose);
    }
  }

  /** A fallback strategy that selects the next scoring setpoint (A–L) along the sequence. */
  public class FallbackNextAlong extends FallbackPlan {

    /**
     * Returns the next scoring Pose2d setpoint after the original target, wrapping around if
     * needed.
     *
     * @param originalTarget The original target pose.
     * @param currentPose The current pose of the robot.
     * @return The next available scoring setpoint pose.
     */
    @Override
    public Pose2d calculate(Pose2d originalTarget, Pose2d currentPose) {
      Setpoints.SetpointsReefscape[] points = Setpoints.SetpointsReefscape.values();

      int startIdx = -1;
      for (int i = 0; i < points.length; i++) {
        Setpoints.SetpointsReefscape sp = points[i];
        if (sp.type() == Setpoints.SetpointType.kScore
            && sp.getPose(0, 0, 0.3, 0.2).equals(originalTarget)) {
          startIdx = i;
          break;
        }
      }

      if (startIdx == -1) return originalTarget;

      for (int offset = 1; offset < points.length; offset++) {
        int idx = (startIdx + offset) % points.length;
        if (points[idx].type() == Setpoints.SetpointType.kScore) {
          return points[idx].getPose(currentPose.getX(), currentPose.getY(), 0.3, 0.2);
        }
      }

      return originalTarget;
    }
  }

  public static class Navigation {}

  public static Navigation get_path_to(NavigationType t) {
    return new Navigation();
  }
}
