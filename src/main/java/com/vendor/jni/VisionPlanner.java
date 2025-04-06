package com.vendor.jni;

import com.vendor.jni.FieldPlanner.Obstacle;
import com.vendor.jni.Vision.Vision;
import com.vendor.jni.Vision.Vision.ObstacleType;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class VisionPlanner {
  public static class VisionObstacle extends Obstacle {
    public Translation2d loc;
    public double sizeX;
    public double sizeY;

    public VisionObstacle(Translation2d loc, double strength, ObstacleType type) {
      super(strength, true);
      this.loc = loc;
      this.sizeX = type.getSize().getFirst();
      this.sizeY = type.getSize().getSecond();
    }

    @Override
    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var distance = loc.getDistance(position);
      if (distance > 3) return new Force();

      var scaledRadius = Math.max(sizeX, sizeY) / 2;
      var mag = distToForceMag(distance - scaledRadius);
      return new Force(mag, position.minus(loc).getAngle());
    }

    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      // 1. Check if center of obstacle is inside robot rectangle
      if (FieldPlanner.isPointInPolygon(loc, rectCorners)) return true;

      // 2. Check if any robot corner is inside the vision obstacle ellipse
      double rx = sizeX / 2;
      double ry = sizeY / 2;
      for (Translation2d corner : rectCorners) {
        double dx = corner.getX() - loc.getX();
        double dy = corner.getY() - loc.getY();
        if ((dx * dx) / (rx * rx) + (dy * dy) / (ry * ry) <= 1) {
          return true;
        }
      }

      // 3. Check if any edge of robot intersects the ellipse (approximate with bounding circle)
      double boundingRadius = Math.max(rx, ry);
      for (int i = 0; i < rectCorners.length; i++) {
        Translation2d a = rectCorners[i];
        Translation2d b = rectCorners[(i + 1) % rectCorners.length];
        if (FieldPlanner.distanceFromPointToSegment(loc, a, b) < boundingRadius) return true;
      }

      return false;
    }
  }

  private List<Vision> m_vision = new ArrayList<Vision>();

  public VisionPlanner() {}

  public VisionPlanner withVision(Vision vision) {
    m_vision.add(vision);
    return this;
  }

  public void addVision(Vision vision) {
    m_vision.add(vision);
  }

  public List<VisionObstacle> getObstacles() {
    return m_vision.stream()
        .flatMap(v -> java.util.Arrays.stream(v.getObstacles()))
        .map(o -> new VisionObstacle(new Translation2d(o.x(), o.y()), 1.5, o.type()))
        .collect(Collectors.toList());
  }
}
