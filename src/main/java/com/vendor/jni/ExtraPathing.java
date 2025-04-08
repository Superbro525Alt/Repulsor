package com.vendor.jni;

import com.vendor.jni.FieldPlanner.Obstacle;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.function.Predicate;

public class ExtraPathing {
  public enum NavigationType {
    kMoveObstacles
  }

  public static boolean isClearPath(
      Translation2d start,
      Translation2d goal,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters) {

    Pose2d pose = new Pose2d(start, new Rotation2d());
    double stepTime = 0.02;
    int maxSteps = 1000;

    for (int i = 0; i < maxSteps; i++) {
      Force goalForce = computeGoalForce(pose.getTranslation(), goal);
      Force obstacleForce = computeObstacleForce(pose.getTranslation(), obstacles);
      Force netForce = goalForce.plus(obstacleForce);

      double maxSpeed = 2.0;
      double netMag = netForce.getNorm();
      Rotation2d netDir = netForce.getAngle();
      double speed = Math.min(netMag, maxSpeed);
      double dx = speed * netDir.getCos() * stepTime;
      double dy = speed * netDir.getSin() * stepTime;

      if (Math.hypot(dx, dy) < 1e-4) {
        return false;
      }

      Translation2d nextPos = pose.getTranslation().plus(new Translation2d(dx, dy));
      Translation2d[] corners =
          getRobotCorners(nextPos, pose.getRotation(), robotLengthMeters, robotWidthMeters);

      for (Obstacle obs : obstacles) {
        if (obs.intersectsRectangle(corners)) {
          return false;
        }
      }

      if (nextPos.getDistance(goal) < 0.1) {
        return true;
      }

      pose = new Pose2d(nextPos, pose.getRotation());
    }
    return false;
  }

  private static Force computeGoalForce(Translation2d currentPos, Translation2d goal) {
    Translation2d d = goal.minus(currentPos);
    double dist = d.getNorm();
    if (dist < 1e-9) {
      return Force.kZero;
    }
    double mag = 1.2 / (0.0001 + dist * dist);
    return new Force(mag, d.getAngle());
  }

  private static Force computeObstacleForce(
      Translation2d currentPos, List<? extends Obstacle> obstacles) {
    Force total = Force.kZero;
    for (Obstacle obs : obstacles) {
      total = total.plus(obs.getForceAtPosition(currentPos, Translation2d.kZero));
    }
    return total;
  }

  private static Translation2d[] getRobotCorners(
      Translation2d center, Rotation2d heading, double length, double width) {

    double halfLength = length / 2.0;
    double halfWidth = width / 2.0;
    Translation2d[] corners = {
      new Translation2d(-halfLength, -halfWidth),
      new Translation2d(halfLength, -halfWidth),
      new Translation2d(halfLength, halfWidth),
      new Translation2d(-halfLength, halfWidth)
    };
    for (int i = 0; i < corners.length; i++) {
      corners[i] = corners[i].rotateBy(heading).plus(center);
    }
    return corners;
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
    public abstract Pose2d calculate(
        Pose2d originalTarget,
        Pose2d currentPose,
        Predicate<Translation2d> isClear,
        boolean isBouncing,
        List<Pose2d> avoid);

    /**
     * Calculates a new fallback target pose using a Translation2d as the original target.
     *
     * @param originalTarget The original target location as a Translation2d.
     * @param currentPose The current pose of the robot.
     * @return A new Pose2d representing the fallback target.
     */
    public Pose2d calculate(
        Translation2d originalTarget,
        Pose2d currentPose,
        Predicate<Translation2d> func,
        boolean isBouncing,
        List<Pose2d> avoid) {
      return calculate(
          new Pose2d(originalTarget.getX(), originalTarget.getY(), new Rotation2d()),
          currentPose,
          func,
          isBouncing,
          avoid);
    }
  }

  /** A fallback strategy that selects the next scoring setpoint (A–L) along the sequence. */
  public class FallbackNextAlong extends FallbackPlan {
    private double m_robot_x;
    private double m_robot_y;
    private double m_coral_offset;
    private double m_algae_offset;
    private Queue<Pose2d> lastPoses = new LinkedList<Pose2d>();

    public FallbackNextAlong(
        double robot_x, double robot_y, double coral_offset, double algae_offset) {
      m_robot_x = robot_x;
      m_robot_y = robot_y;
      m_coral_offset = coral_offset;
      m_algae_offset = algae_offset;
    }

    /**
     * Returns the next scoring Pose2d setpoint after the original target, wrapping around if
     * needed.
     *
     * @param originalTarget The original target pose.
     * @param currentPose The current pose of the robot.
     * @return The next available scoring setpoint pose.
     */
    @Override
    public Pose2d calculate(
        Pose2d originalTarget,
        Pose2d currentPose,
        Predicate<Translation2d> isClear,
        boolean isBouncing,
        List<Pose2d> avoid) {
      Setpoints.SetpointsReefscape[] points = Setpoints.SetpointsReefscape.values();

      int startIdx = -1;
      for (int i = 0; i < points.length; i++) {
        Setpoints.SetpointsReefscape sp = points[i];
        if (sp.type() == Setpoints.SetpointType.kScore
            && sp.getPose(m_robot_x, m_robot_y, m_coral_offset, m_algae_offset)
                .getTranslation()
                .equals(originalTarget.getTranslation())) {
          startIdx = i;
          break;
        }
      }

      if (startIdx == -1) return originalTarget;

      for (int offset = 1; offset < points.length; offset++) {
        int idx = offset % points.length;
        if (points[idx].type() == Setpoints.SetpointType.kScore) {
          Pose2d pose = points[idx].getPose(m_robot_x, m_robot_y, m_coral_offset, m_algae_offset);
          // System.out.println(pose.toString());
          // System.out.println(isClear.test(pose.getTranslation()));
          if (isClear.test(pose.getTranslation()) && !avoid.contains(pose)) {
            lastPoses.add(pose);
            if (lastPoses.size() > 20) {
              lastPoses.poll();
            }

            return pose;
          }
        }
      }

      lastPoses.add(originalTarget);

      if (lastPoses.size() > 20) {
        lastPoses.poll();
      }

      return originalTarget;
    }
  }

  public static class Navigation {}

  public static Navigation get_path_to(NavigationType t) {
    return new Navigation();
  }

  public class BounceListener {
    private final double bounceDistanceThreshold;
    private final int bounceHistoryLimit;
    private final Queue<Pose2d> recentGoals = new LinkedList<>();
    private boolean isBouncing;

    public BounceListener(double bounceDistanceThreshold, int bounceHistoryLimit) {
      this.bounceDistanceThreshold = bounceDistanceThreshold;
      this.bounceHistoryLimit = bounceHistoryLimit;
    }

    public void update(Pose2d currentGoal) {
      recentGoals.add(currentGoal);
      if (recentGoals.size() > bounceHistoryLimit) {
        recentGoals.poll();
      }

      isBouncing = checkBouncing();
    }

    private boolean checkBouncing() {
      if (recentGoals.size() < bounceHistoryLimit) return false;

      int similarCount = 0;
      Pose2d[] goals = recentGoals.toArray(new Pose2d[0]);
      for (int i = 0; i < goals.length - 1; i++) {
        for (int j = i + 1; j < goals.length; j++) {
          if (goals[i].getTranslation().getDistance(goals[j].getTranslation())
              < bounceDistanceThreshold) {
            similarCount++;
          }
        }
      }

      int totalPairs = (goals.length * (goals.length - 1)) / 2;
      return similarCount >= (totalPairs * 0.6);
    }

    public void clearHistory() {
      recentGoals.clear();
      isBouncing = false;
    }

    public boolean isBouncing() {
      return isBouncing;
    }
  }
}
