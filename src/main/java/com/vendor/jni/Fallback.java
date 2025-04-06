package com.vendor.jni;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Fallback {
  public abstract class PlannerFallback {
    public abstract ChassisSpeeds calculate(Translation2d currentPose, Translation2d target);

    public abstract boolean within(Translation2d err);
  }

  public class PID extends PlannerFallback {
    private PIDController xController;
    private PIDController yController;

    public PID(double kP, double kI, double kD) {
      xController = new PIDController(kP, kI, kD);
      yController = new PIDController(kP, kI, kD);
    }

    @Override
    public ChassisSpeeds calculate(Translation2d currentPose, Translation2d target) {
      return new ChassisSpeeds(
          xController.calculate(currentPose.getX(), target.getX()),
          yController.calculate(currentPose.getY(), target.getY()),
          0);
    }

    @Override
    public boolean within(Translation2d err) {
      return err.getNorm() < 0.02;
    }
  }
}
