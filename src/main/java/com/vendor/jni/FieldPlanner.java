package com.vendor.jni;

import static edu.wpi.first.units.Units.*;

import com.vendor.jni.ExtraPathing.FallbackPlan;
import com.vendor.jni.Fallback.PlannerFallback;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class FieldPlanner {
  public abstract static class Obstacle {
    double strength = 1.0;
    boolean positive = true;

    public Obstacle(double strength, boolean positive) {
      this.strength = strength;
      this.positive = positive;
    }

    public abstract Force getForceAtPosition(Translation2d position, Translation2d target);

    protected double distToForceMag(double dist) {
      var forceMag = strength / (0.00001 + Math.abs(dist * dist));
      forceMag *= positive ? 1 : -1;
      return forceMag;
    }

    protected double distToForceMag(double dist, double falloff) {
      var original = strength / (0.00001 + Math.abs(dist * dist));
      var falloffMag = strength / (0.00001 + Math.abs(falloff * falloff));
      return Math.max(original - falloffMag, 0) * (positive ? 1 : -1);
    }

    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      return false;
    }
  }

  static class PointObstacle extends Obstacle {
    Translation2d loc;
    double radius = 0.5;

    public PointObstacle(Translation2d loc, double strength, boolean positive) {
      super(strength, positive);
      this.loc = loc;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = loc.getDistance(position);
      if (dist > 4) {
        return new Force();
      }
      var outwardsMag = distToForceMag(loc.getDistance(position) - radius);
      var initial = new Force(outwardsMag, position.minus(loc).getAngle());
      var theta = target.minus(position).getAngle().minus(position.minus(loc).getAngle());
      double mag = outwardsMag * Math.signum(Math.sin(theta.getRadians() / 2)) / 2;

      return initial
          .rotateBy(Rotation2d.kCCW_90deg)
          .div(initial.getNorm())
          .times(mag)
          .plus(initial);
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      // 1. Check if center is inside robot rectangle
      if (FieldPlanner.isPointInPolygon(loc, rectCorners)) return true;

      // 2. Check if any corner is inside the obstacle
      for (Translation2d corner : rectCorners) {
        if (corner.getDistance(loc) < radius) return true;
      }

      // 3. Check if any edge intersects the circle
      for (int i = 0; i < rectCorners.length; i++) {
        Translation2d a = rectCorners[i];
        Translation2d b = rectCorners[(i + 1) % rectCorners.length];
        if (FieldPlanner.distanceFromPointToSegment(loc, a, b) < radius) return true;
      }

      return false;
    }
  }

  static class SnowmanObstacle extends Obstacle {
    Translation2d loc;
    double radius = 0.5;

    public SnowmanObstacle(Translation2d loc, double strength, double radius, boolean positive) {
      super(strength, positive);
      this.loc = loc;
      this.radius = radius;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var targetToLoc = loc.minus(target);
      var targetToLocAngle = targetToLoc.getAngle();
      // 1 meter away from loc, opposite target.
      var sidewaysCircle = new Translation2d(1, targetToLoc.getAngle()).plus(loc);
      var sidewaysMag = distToForceMag(sidewaysCircle.getDistance(position));
      var outwardsMag = distToForceMag(Math.max(0.01, loc.getDistance(position) - radius));
      var initial = new Force(outwardsMag, position.minus(loc).getAngle());

      // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
      var sidewaysTheta =
          target.minus(position).getAngle().minus(position.minus(sidewaysCircle).getAngle());

      double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
      var sidewaysAngle = targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg);
      return new Force(sideways, sidewaysAngle).plus(initial);
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      // Same logic as PointObstacle
      if (FieldPlanner.isPointInPolygon(loc, rectCorners)) return true;

      for (Translation2d corner : rectCorners) {
        if (corner.getDistance(loc) < radius) return true;
      }

      for (int i = 0; i < rectCorners.length; i++) {
        Translation2d a = rectCorners[i];
        Translation2d b = rectCorners[(i + 1) % rectCorners.length];
        if (FieldPlanner.distanceFromPointToSegment(loc, a, b) < radius) return true;
      }

      return false;
    }
  }

  public static class HorizontalObstacle extends Obstacle {
    public double y;

    public HorizontalObstacle(double y, double strength, boolean positive) {
      super(strength, positive);
      this.y = y;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      return new Force(0, distToForceMag(y - position.getY(), 1));
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      for (int i = 0; i < rectCorners.length; i++) {
        Translation2d a = rectCorners[i];
        if (Math.abs(a.getY() - y) < 0.1) return true; // tolerance for "collision"
      }
      return false;
    }
  }

  public static class VerticalObstacle extends Obstacle {
    public double x;

    public VerticalObstacle(double x, double strength, boolean positive) {
      super(strength, positive);
      this.x = x;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      return new Force(distToForceMag(x - position.getX(), 1), 0);
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      for (int i = 0; i < rectCorners.length; i++) {
        Translation2d a = rectCorners[i];
        if (Math.abs(a.getX() - x) < 0.1) return true; // tolerance for "collision"
      }
      return false;
    }
  }

  public static class TeardropObstacle extends Obstacle {
    public final Translation2d loc;
    public final double primaryMaxRange;
    public final double primaryRadius;
    public final double tailStrength;
    public final double tailLength;

    public TeardropObstacle(
        Translation2d loc,
        double primaryStrength,
        double primaryMaxRange,
        double primaryRadius,
        double tailStrength,
        double tailLength) {
      super(primaryStrength, true);
      this.loc = loc;
      this.primaryMaxRange = primaryMaxRange;
      this.primaryRadius = primaryRadius;
      this.tailStrength = tailStrength;
      this.tailLength = tailLength + primaryMaxRange;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var targetToLoc = loc.minus(target);
      var targetToLocAngle = targetToLoc.getAngle();
      var sidewaysPoint = new Translation2d(tailLength, targetToLoc.getAngle()).plus(loc);

      var positionToLocation = position.minus(loc);
      var positionToLocationDistance = positionToLocation.getNorm();
      Translation2d outwardsForce;
      if (positionToLocationDistance <= primaryMaxRange) {
        outwardsForce =
            new Translation2d(
                distToForceMag(
                    Math.max(positionToLocationDistance - primaryRadius, 0),
                    primaryMaxRange - primaryRadius),
                positionToLocation.getAngle());
      } else {
        outwardsForce = Translation2d.kZero;
      }

      var positionToLine = position.minus(loc).rotateBy(targetToLocAngle.unaryMinus());
      var distanceAlongLine = positionToLine.getX();

      Translation2d sidewaysForce;
      var distanceScalar = distanceAlongLine / tailLength;
      if (distanceScalar >= 0 && distanceScalar <= 1) {
        var secondaryMaxRange =
            MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
        var distanceToLine = Math.abs(positionToLine.getY());
        if (distanceToLine <= secondaryMaxRange) {
          double strength;
          if (distanceAlongLine < primaryMaxRange) {
            strength = tailStrength * (distanceAlongLine / primaryMaxRange);
          } else {
            strength =
                -tailStrength * distanceAlongLine / (tailLength - primaryMaxRange)
                    + tailLength * tailStrength / (tailLength - primaryMaxRange);
          }
          strength *= 1 - distanceToLine / secondaryMaxRange;

          var sidewaysMag = tailStrength * strength * (secondaryMaxRange - distanceToLine);
          // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
          var sidewaysTheta =
              target.minus(position).getAngle().minus(position.minus(sidewaysPoint).getAngle());
          sidewaysForce =
              new Translation2d(
                  sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians())),
                  targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg));
        } else {
          sidewaysForce = Translation2d.kZero;
        }
      } else {
        sidewaysForce = Translation2d.kZero;
      }

      return new Force(
          outwardsForce.plus(sidewaysForce).getNorm(),
          outwardsForce.plus(sidewaysForce).getAngle());
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      // Circle check at primary part
      if (FieldPlanner.isPointInPolygon(loc, rectCorners)) return true;
      for (Translation2d corner : rectCorners) {
        if (corner.getDistance(loc) < primaryMaxRange) return true;
      }
      for (int i = 0; i < rectCorners.length; i++) {
        Translation2d a = rectCorners[i];
        Translation2d b = rectCorners[(i + 1) % rectCorners.length];
        if (FieldPlanner.distanceFromPointToSegment(loc, a, b) < primaryMaxRange) return true;
      }

      // Tail collision region (simplified as a swept box behind the obstacle)
      Rotation2d tailDir = new Rotation2d(); // Facing +X (0 radians)
      Translation2d tailStart = loc;
      Translation2d tailEnd = tailStart.plus(new Translation2d(tailLength, tailDir));

      for (Translation2d corner : rectCorners) {
        Translation2d delta = tailEnd.minus(tailStart);
        Translation2d unit = delta.div(delta.getNorm()); // normalize

        double projection = dot(corner.minus(tailStart), unit);

        if (projection >= 0 && projection <= tailLength) {
          double lateral =
              Math.abs((corner.minus(tailStart)).rotateBy(tailDir.unaryMinus()).getY());
          if (lateral < primaryMaxRange) {
            return true;
          }
        }
      }

      return false;
    }
  }

  public static final double GOAL_STRENGTH = 1.2;

  public static final List<Obstacle> FIELD_OBSTACLES =
      List.of(
          new TeardropObstacle(new Translation2d(4.49, 4), 1.2, 2.2, 1.03, 3, 2),
          new TeardropObstacle(new Translation2d(13.08, 4), 1.2, 2.2, 1.03, 3, 2));

  public static final List<Obstacle> WALLS =
      List.of(
          new HorizontalObstacle(0.0, 2, true),
          new HorizontalObstacle(Constants.FIELD_WIDTH, 1.4, false),
          new VerticalObstacle(0.0, 2, true),
          new VerticalObstacle(Constants.FIELD_LENGTH, 1.4, false));

  private List<Obstacle> fixedObstacles = new ArrayList<>();
  private Translation2d goal = Translation2d.kZero;

  private static final int ARROWS_X = RobotBase.isSimulation() ? 40 : 0;
  private static final int ARROWS_Y = RobotBase.isSimulation() ? 20 : 0;
  private static final int ARROWS_SIZE = (ARROWS_X + 1) * (ARROWS_Y + 1);

  private ArrayList<Pose2d> arrows = new ArrayList<>(ARROWS_SIZE);

  private Optional<Distance> currentErr = Optional.empty();
  private Optional<PlannerFallback> fallback = Optional.empty();
  private Optional<FallbackPlan> onBlocked = Optional.empty();

  public FieldPlanner() {
    fixedObstacles.addAll(FIELD_OBSTACLES);
    fixedObstacles.addAll(WALLS);
    for (int i = 0; i < ARROWS_SIZE; i++) {
      arrows.add(new Pose2d());
    }
  }

  public List<Obstacle> getObstacles() {
    return fixedObstacles;
  }

  public Translation2d getGoal() {
    return goal;
  }

  public FieldPlanner withFallbackPlan(FallbackPlan _fallback) {
    onBlocked = Optional.of(_fallback);
    return this;
  }

  public FieldPlanner withFallback(PlannerFallback _fallback) {
    fallback = Optional.of(_fallback);
    return this;
  }

  private Pose2d arrowBackstage = new Pose2d(-10, -10, Rotation2d.kZero);

  // A grid of arrows drawn in AScope
  public void updateArrows(List<? extends Obstacle> dynamicObstacles) {
    for (int x = 0; x <= ARROWS_X; x++) {
      for (int y = 0; y <= ARROWS_Y; y++) {
        var translation =
            new Translation2d(
                x * Constants.FIELD_LENGTH / ARROWS_X, y * Constants.FIELD_WIDTH / ARROWS_Y);
        var force = Force.kZero;
        force = force.plus(getObstacleForce(translation, goal, dynamicObstacles));
        force = force.plus(getWallForce(translation, goal));
        force = force.plus(getGoalForce(translation, goal));
        if (force.getNorm() < 1e-6) {
          arrows.set(x * (ARROWS_Y + 1) + y, arrowBackstage);
        } else {
          var rotation = force.getAngle();

          arrows.set(x * (ARROWS_Y + 1) + y, new Pose2d(translation, rotation));
        }
      }
    }
  }

  public ArrayList<Pose2d> getArrows() {
    return arrows;
  }

  Force getGoalForce(Translation2d curLocation, Translation2d goal) {
    var displacement = goal.minus(curLocation);
    if (displacement.getNorm() == 0) {
      return new Force();
    }
    var direction = displacement.getAngle();
    var mag =
        GOAL_STRENGTH * (1 + 1.0 / (0.0001 + displacement.getNorm() * displacement.getNorm()));
    return new Force(mag, direction);
  }

  Force getWallForce(Translation2d curLocation, Translation2d target) {
    var force = Force.kZero;
    for (Obstacle obs : WALLS) {
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    }
    return force;
  }

  Force getObstacleForce(
      Translation2d curLocation, Translation2d target, List<? extends Obstacle> extra) {
    var force = Force.kZero;
    for (Obstacle obs : FIELD_OBSTACLES) {
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    }

    for (Obstacle obs : extra) {
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    }

    return force;
  }

  Force getObstacleForce(Translation2d curLocation, Translation2d target) {
    var force = Force.kZero;
    for (Obstacle obs : FIELD_OBSTACLES) {
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    }
    return force;
  }

  Force getForce(Translation2d curLocation, Translation2d target) {
    var goalForce =
        getGoalForce(curLocation, target)
            .plus(getObstacleForce(curLocation, target))
            .plus(getWallForce(curLocation, target))
            .times(Math.min(1.0, curLocation.getDistance(target))); // Adjust scaling
    return goalForce;
  }

  public static class RepulsorSample {
    private Translation2d m_goal;
    private LinearVelocity m_vx;
    private LinearVelocity m_vy;

    public RepulsorSample(Translation2d goal, double vx, double vy) {
      m_goal = goal;
      m_vx = MetersPerSecond.of(vx);
      m_vy = MetersPerSecond.of(vy);
    }

    public RepulsorSample(Translation2d goal, ChassisSpeeds v) {
      m_goal = goal;
      m_vx = MetersPerSecond.of(v.vxMetersPerSecond);
      m_vy = MetersPerSecond.of(v.vyMetersPerSecond);
    }

    public ChassisSpeeds asChassisSpeeds(AngularVelocity omega, Rotation2d currentRot) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(m_vx, m_vy, omega, currentRot);
    }

    public Translation2d goal() {
      return m_goal;
    }
  }

  public void setGoal(Translation2d goal) {
    this.goal = goal;
  }

  public Optional<Distance> getErr() {
    return currentErr;
  }

  public RepulsorSample calculate(
      Pose2d pose, List<? extends Obstacle> dynamicObstacles, double robot_x, double robot_y) {
    if (!ExtraPathing.isClearPath(
        pose.getTranslation(), goal, dynamicObstacles, robot_x, robot_y)) {
      if (onBlocked.isEmpty()) {
        return new RepulsorSample(pose.getTranslation(), 0, 0);
      }

      setGoal(onBlocked.get().calculate(goal, pose).getTranslation());
      System.out.println(onBlocked.get().calculate(goal, pose).toString());
      return calculate(pose, dynamicObstacles, robot_x, robot_y);
    }

    updateArrows(dynamicObstacles);
    double stepSize_m = 0.02;
    var curTrans = pose.getTranslation();
    var err = curTrans.minus(goal);

    currentErr = Optional.of(Meters.of(err.getNorm()));

    if (err.getNorm() < stepSize_m) {
      if (fallback.isEmpty()) {
        return new RepulsorSample(pose.getTranslation(), 0, 0);
      }
      if (fallback.get().within(err)) {
        return new RepulsorSample(pose.getTranslation(), 0, 0);
      }
      return new RepulsorSample(
          pose.getTranslation(), fallback.get().calculate(pose.getTranslation(), goal));
    } else {
      var obstacleForce =
          getObstacleForce(curTrans, goal, dynamicObstacles).plus(getWallForce(curTrans, goal));
      var netForce = obstacleForce;
      netForce = getGoalForce(curTrans, goal).plus(netForce);
      var dist = err.getNorm();
      stepSize_m = Math.min(5.14, Math.sqrt(6 /* 14 */ * dist)) * 0.02;
      var step = new Translation2d(stepSize_m, netForce.getAngle());
      return new RepulsorSample(goal, (step.getX() / 0.02), (step.getY() / 0.02));
    }
  }

  public static boolean isPointInPolygon(Translation2d point, Translation2d[] polygon) {
    int crossings = 0;
    for (int i = 0; i < polygon.length; i++) {
      Translation2d a = polygon[i];
      Translation2d b = polygon[(i + 1) % polygon.length];
      boolean cond1 = (a.getY() > point.getY()) != (b.getY() > point.getY());
      double slope =
          (b.getX() - a.getX()) * (point.getY() - a.getY()) / (b.getY() - a.getY()) + a.getX();
      if (cond1 && point.getX() < slope) {
        crossings++;
      }
    }
    return (crossings % 2 == 1);
  }

  public static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  public static double distanceFromPointToSegment(
      Translation2d p, Translation2d a, Translation2d b) {
    Translation2d ap = p.minus(a);
    Translation2d ab = b.minus(a);
    double abLenSquared = ab.getNorm() * ab.getNorm();
    if (abLenSquared == 0) return ap.getNorm();
    double t = Math.max(0, Math.min(1, dot(ap, ab) / abLenSquared));
    Translation2d projection = a.plus(ab.times(t));
    return p.getDistance(projection);
  }
}
