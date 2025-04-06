package com.vendor.jni;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public final class Constants {
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static final double FIELD_LENGTH = aprilTagLayout.getFieldLength();
  public static final double FIELD_WIDTH = aprilTagLayout.getFieldWidth();
}
