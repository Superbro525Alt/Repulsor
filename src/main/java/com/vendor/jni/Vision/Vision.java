package com.vendor.jni.Vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;

public interface Vision {
  public static class ObstacleType {
    private Pair<Double, Double> size;

    ObstacleType(double size_x, double size_y) {
      size = new Pair<Double, Double>(size_x, size_y);
    }

    public Pair<Double, Double> getSize() {
      return size;
    }
  }

  public static class Obstacle {
    private double m_x;
    private double m_y;
    private ObstacleType m_type;

    public Obstacle(double x, double y, ObstacleType type) {
      m_x = x;
      m_y = y;
      m_type = type;
    }

    public Obstacle(Pose2d pose, ObstacleType type) {
      m_x = pose.getX();
      m_y = pose.getY();
      m_type = type;
    }

    public double x() {
      return m_x;
    }

    public double y() {
      return m_y;
    }

    public ObstacleType type() {
      return m_type;
    }
  }

  public default Obstacle[] getObstacles() {
    return new Obstacle[0];
  }
}
