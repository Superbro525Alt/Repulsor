package com.vendor.jni;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Drive extends SubsystemBase {
  public abstract void runVelocity(ChassisSpeeds speeds);

  public abstract Pose2d getPose();

  public abstract PIDController getOmegaPID();
}
