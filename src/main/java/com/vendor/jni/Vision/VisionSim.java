package com.vendor.jni.Vision;

import com.vendor.jni.Setpoints.SetpointsReefscape;

public class VisionSim implements Vision {
  @Override
  public Obstacle[] getObstacles() {
    return new Obstacle[] {
      new Vision.Obstacle(
          SetpointsReefscape.A.getPose(0, 0, 0, 0), new Vision.ObstacleType(0.5, 0.5))
    };
  }
}
