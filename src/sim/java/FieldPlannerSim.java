import static edu.wpi.first.units.Units.Radians;

import com.vendor.jni.*;
import com.vendor.jni.Setpoints.HeightSetpoint;
import com.vendor.jni.Setpoints.RepulsorSetpoint;
import com.vendor.jni.Setpoints.SetpointsReefscape;
import com.vendor.jni.Vision.VisionSim;
import com.vendor.jni.VisionPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import processing.core.PApplet;

public class FieldPlannerSim extends PApplet {

  Repulsor planner;
  Pose2d robotPose = new Pose2d(2, 2, new Rotation2d());
  boolean draggingGoal = false;
  boolean useKeyboard = false;
  RepulsorSetpoint goal = new RepulsorSetpoint(SetpointsReefscape.A, HeightSetpoint.L2);

  public static void main(String[] args) {
    PApplet.main("FieldPlannerSim");
  }

  @Override
  public void settings() {
    size(900, 450);
  }

  @Override
  public void setup() {
    planner =
        new Repulsor(
                new Drive() {
                  public Pose2d getPose() {
                    return robotPose;
                  }

                  public PIDController getOmegaPID() {
                    return new PIDController(0.1, 0, 0.01);
                  }

                  public void runVelocity(ChassisSpeeds speeds) {
                    if (!useKeyboard && !draggingGoal) {
                      double dt = 0.02;
                      Translation2d delta =
                          new Translation2d(
                              speeds.vxMetersPerSecond * dt, speeds.vyMetersPerSecond * dt);
                      Rotation2d deltaRot =
                          new Rotation2d(
                              Radians.of(
                                  (speeds.omegaRadiansPerSecond * dt)
                                      + robotPose.getRotation().getRadians()));

                      robotPose = new Pose2d(robotPose.getTranslation().plus(delta), deltaRot);
                    }
                  }
                },
                0.5,
                0.5,
                0.35,
                0.12)
            .withFallback(new Fallback().new PID(1, 0, 0))
            .withVision(new VisionSim());
  }

  @Override
  public void draw() {
    background(255);
    float scaleX = width / (float) com.vendor.jni.Constants.FIELD_LENGTH;
    float scaleY = height / (float) com.vendor.jni.Constants.FIELD_WIDTH;

    // Handle keyboard movement
    useKeyboard = false;
    double moveSpeed = 0.05;
    if (!draggingGoal && !keyPressed) {
      if (keyPressed) {
        useKeyboard = true;
        double dx = 0, dy = 0;
        if (key == 'w') dy += moveSpeed;
        if (key == 's') dy -= moveSpeed;
        if (key == 'a') dx -= moveSpeed;
        if (key == 'd') dx += moveSpeed;

        if (keyCode == UP) dy += moveSpeed;
        if (keyCode == DOWN) dy -= moveSpeed;
        if (keyCode == LEFT) dx -= moveSpeed;
        if (keyCode == RIGHT) dx += moveSpeed;

        Translation2d updated = robotPose.getTranslation().plus(new Translation2d(dx, dy));
        robotPose = new Pose2d(updated, robotPose.getRotation());
      }
    }

    planner.alignTo(goal, new Trigger(() -> false)).execute();

    // Handle number key goal selection
    double robotX = robotPose.getX();
    double robotY = robotPose.getY();
    double coralOffset = 0.3;
    double algaeOffset = 0.2;

    if (keyPressed && !draggingGoal) {
      switch (key) {
        case '1' -> setGoal(SetpointsReefscape.A, robotX, robotY, coralOffset, algaeOffset);
        case '2' -> setGoal(SetpointsReefscape.B, robotX, robotY, coralOffset, algaeOffset);
        case '3' -> setGoal(SetpointsReefscape.C, robotX, robotY, coralOffset, algaeOffset);
        case '4' -> setGoal(SetpointsReefscape.D, robotX, robotY, coralOffset, algaeOffset);
        case '5' -> setGoal(SetpointsReefscape.E, robotX, robotY, coralOffset, algaeOffset);
        case '6' -> setGoal(SetpointsReefscape.F, robotX, robotY, coralOffset, algaeOffset);
        case '7' -> setGoal(SetpointsReefscape.G, robotX, robotY, coralOffset, algaeOffset);
        case '8' -> setGoal(SetpointsReefscape.H, robotX, robotY, coralOffset, algaeOffset);
        case '9' -> setGoal(SetpointsReefscape.I, robotX, robotY, coralOffset, algaeOffset);
        case '0' -> setGoal(SetpointsReefscape.J, robotX, robotY, coralOffset, algaeOffset);
        case 'q' -> setGoal(SetpointsReefscape.K, robotX, robotY, coralOffset, algaeOffset);
        case 'w' -> setGoal(SetpointsReefscape.L, robotX, robotY, coralOffset, algaeOffset);
      }
    }

    // Update repulsor

    // --- Arrows ---
    fill(0);
    for (Pose2d arrow : planner.getFieldPlanner().getArrows()) {
      Translation2d pos = arrow.getTranslation();
      if (pos.getX() == -10 && pos.getY() == -10) continue;

      float x = (float) pos.getX() * scaleX;
      float y = height - (float) pos.getY() * scaleY;

      float len = 10;
      float angle = (float) arrow.getRotation().getRadians();
      float x2 = x + len * cos(angle);
      float y2 = y - len * sin(angle);

      stroke(0);
      line(x, y, x2, y2);
      ellipse(x, y, 4, 4);
    }

    // --- Goal ---
    Translation2d goal = planner.getFieldPlanner().getGoal();
    fill(255, 0, 0);
    ellipse((float) goal.getX() * scaleX, height - (float) goal.getY() * scaleY, 10, 10);

    // --- Obstacles ---
    for (FieldPlanner.Obstacle obs : planner.getFieldPlanner().getObstacles()) {
      if (obs instanceof FieldPlanner.TeardropObstacle teardrop) {
        Translation2d loc = teardrop.loc;
        float radius = (float) teardrop.primaryRadius * scaleX;
        fill(255, 165, 0);
        ellipse(
            (float) loc.getX() * scaleX,
            height - (float) loc.getY() * scaleY,
            radius * 2,
            radius * 2);
      } else if (obs instanceof FieldPlanner.HorizontalObstacle ho) {
        float y = height - (float) ho.y * scaleY;
        fill(128);
        rect(0, y - 2, width, 4);
      } else if (obs instanceof FieldPlanner.VerticalObstacle vo) {
        float x = (float) vo.x * scaleX;
        fill(128);
        rect(x - 2, 0, 4, height);
      }
    }

    for (FieldPlanner.Obstacle obs : planner.getVisionPlanner().getObstacles()) {
      if (obs instanceof VisionPlanner.VisionObstacle vo) {
        Translation2d loc = vo.loc;
        float x = (float) loc.getX() * scaleX;
        float y = height - (float) loc.getY() * scaleY;
        float radiusX = (float) vo.sizeX * scaleX;
        float radiusY = (float) vo.sizeY * scaleY;

        fill(0, 0, 255, 100);
        ellipse(x, y, radiusX * 2, radiusY * 2);
      }
    }

    // --- Robot rectangle ---
    float robotLength = 0.8f;
    float robotWidth = 0.7f;
    Translation2d[] corners = {
      new Translation2d(-robotLength / 2, -robotWidth / 2),
      new Translation2d(robotLength / 2, -robotWidth / 2),
      new Translation2d(robotLength / 2, robotWidth / 2),
      new Translation2d(-robotLength / 2, robotWidth / 2)
    };

    Translation2d robotTrans = robotPose.getTranslation();
    Rotation2d robotRot = robotPose.getRotation();

    beginShape();
    fill(0, 255, 0, 80);
    for (Translation2d corner : corners) {
      Translation2d rotated = corner.rotateBy(robotRot).plus(robotTrans);
      float sx = (float) rotated.getX() * scaleX;
      float sy = height - (float) rotated.getY() * scaleY;
      vertex(sx, sy);
    }
    endShape(CLOSE);

    // --- Path status ---
    boolean isClear =
        ExtraPathing.isClearPath(
            robotTrans,
            goal,
            planner.getVisionPlanner().getObstacles(),
            robotLength,
            robotWidth,
            planner.getFieldPlanner());

    fill(isClear ? color(0, 200, 0) : color(200, 0, 0));
    textAlign(LEFT, TOP);
    text("Path " + (isClear ? "CLEAR" : "BLOCKED"), 10, 10);
  }

  private void setGoal(
      SetpointsReefscape setpoint,
      double robotX,
      double robotY,
      double coralOffset,
      double algaeOffset) {
    goal = new RepulsorSetpoint(setpoint, HeightSetpoint.L2);
  }

  @Override
  public void mousePressed() {
    float scaleX = width / (float) com.vendor.jni.Constants.FIELD_LENGTH;
    float scaleY = height / (float) com.vendor.jni.Constants.FIELD_WIDTH;

    Translation2d goal = planner.getFieldPlanner().getGoal();
    float gx = (float) goal.getX() * scaleX;
    float gy = height - (float) goal.getY() * scaleY;

    if (dist(mouseX, mouseY, gx, gy) < 10) {
      draggingGoal = true;
    }
  }
}
