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

  private Repulsor planner;
  private Pose2d robotPose = new Pose2d(2, 2, new Rotation2d());
  private boolean draggingGoal = false;
  private boolean useKeyboard = false;
  private RepulsorSetpoint goal = new RepulsorSetpoint(SetpointsReefscape.A, HeightSetpoint.L2);

  private float scaleX;
  private float scaleY;

  private final Translation2d[] robotCorners = {
    new Translation2d(-0.4, -0.35),
    new Translation2d(0.4, -0.35),
    new Translation2d(0.4, 0.35),
    new Translation2d(-0.4, 0.35)
  };

  private static final double DT = 0.02;
  private static final double KEYBOARD_MOVE_SPEED = 0.05;

  public static void main(String[] args) {
    PApplet.main("FieldPlannerSim");
  }

  @Override
  public void settings() {
    size(900, 450);
  }

  @Override
  public void setup() {
    scaleX = width / (float) com.vendor.jni.Constants.FIELD_LENGTH;
    scaleY = height / (float) com.vendor.jni.Constants.FIELD_WIDTH;

    planner =
        new Repulsor(
                new Drive() {
                  @Override
                  public Pose2d getPose() {
                    return robotPose;
                  }

                  @Override
                  public PIDController getOmegaPID() {
                    return new PIDController(0.1, 0, 0.01);
                  }

                  @Override
                  public void runVelocity(ChassisSpeeds speeds) {
                    if (!useKeyboard && !draggingGoal) {
                      System.out.println(speeds.toString());
                      Translation2d delta =
                          new Translation2d(
                              speeds.vxMetersPerSecond * DT, speeds.vyMetersPerSecond * DT);
                      Rotation2d deltaRot =
                          robotPose
                              .getRotation()
                              .plus(new Rotation2d(speeds.omegaRadiansPerSecond * DT));

                      robotPose = new Pose2d(robotPose.getTranslation().plus(delta), deltaRot);
                    }
                  }
                },
                0.5, 
                0.5, 
                0.35, 
                0.12 
                )
            .withFallback(new Fallback().new PID(1, 0, 0))
            .withVision(new VisionSim());
  }

  @Override
  public void draw() {
    background(255);

    planner.alignTo(goal, new Trigger(() -> false)).execute();

    handleKeyboardMovement();

    handleGoalSelection();

    drawArrows();
    drawGoal();
    drawObstacles();
    drawVisionObstacles();
    drawRobot();

    boolean isClear =
        ExtraPathing.isClearPath(
            robotPose.getTranslation(),
            planner.getFieldPlanner().getGoal(),
            planner.getVisionPlanner().getObstacles(),
            0.8, // robotLength
            0.7 // robotWidth
            );

    fill(isClear ? color(0, 200, 0) : color(200, 0, 0));
    textAlign(LEFT, TOP);
    text("Path " + (isClear ? "CLEAR" : "BLOCKED"), 10, 10);
  }

  private void handleKeyboardMovement() {
    useKeyboard = false;
    if (!draggingGoal && keyPressed) {
      useKeyboard = true;
      double dx = 0, dy = 0;
      if (key == 'w') dy += KEYBOARD_MOVE_SPEED;
      if (key == 's') dy -= KEYBOARD_MOVE_SPEED;
      if (key == 'a') dx -= KEYBOARD_MOVE_SPEED;
      if (key == 'd') dx += KEYBOARD_MOVE_SPEED;

      if (keyCode == UP) dy += KEYBOARD_MOVE_SPEED;
      if (keyCode == DOWN) dy -= KEYBOARD_MOVE_SPEED;
      if (keyCode == LEFT) dx -= KEYBOARD_MOVE_SPEED;
      if (keyCode == RIGHT) dx += KEYBOARD_MOVE_SPEED;

      Translation2d updated = robotPose.getTranslation().plus(new Translation2d(dx, dy));
      robotPose = new Pose2d(updated, robotPose.getRotation());
    }
  }

  private void handleGoalSelection() {
    if (keyPressed && !draggingGoal) {
      switch (key) {
        case '1' -> setGoal(SetpointsReefscape.A);
        case '2' -> setGoal(SetpointsReefscape.B);
        case '3' -> setGoal(SetpointsReefscape.C);
        case '4' -> setGoal(SetpointsReefscape.D);
        case '5' -> setGoal(SetpointsReefscape.E);
        case '6' -> setGoal(SetpointsReefscape.F);
        case '7' -> setGoal(SetpointsReefscape.G);
        case '8' -> setGoal(SetpointsReefscape.H);
        case '9' -> setGoal(SetpointsReefscape.I);
        case '0' -> setGoal(SetpointsReefscape.J);
        case 'q' -> setGoal(SetpointsReefscape.K);
        case 'w' -> setGoal(SetpointsReefscape.L);
        default -> {}
      }
    }
  }

  private void setGoal(SetpointsReefscape setpoint) {
    goal = new RepulsorSetpoint(setpoint, HeightSetpoint.L2);
  }

  public RepulsorSetpoint getGoal() {
    return goal;
  }

  @Override
  public void mousePressed() {
    Translation2d g = planner.getFieldPlanner().getGoal();
    float gx = (float) g.getX() * scaleX;
    float gy = height - (float) g.getY() * scaleY;

    float dx = mouseX - gx;
    float dy = mouseY - gy;
    if ((dx * dx + dy * dy) < 100) { // 10^2
      draggingGoal = true;
    }
  }

  @Override
  public void mouseReleased() {
    draggingGoal = false;
  }

  private void drawArrows() {
    fill(0);
    stroke(0);
    for (Pose2d arrow : planner.getFieldPlanner().getArrows()) {
      Translation2d pos = arrow.getTranslation();
      if (pos.getX() == -10 && pos.getY() == -10) {
        continue;
      }

      float x = (float) pos.getX() * scaleX;
      float y = height - (float) pos.getY() * scaleY;
      float len = 10;
      float angle = (float) arrow.getRotation().getRadians();
      float x2 = x + len * cos(angle);
      float y2 = y - len * sin(angle);

      line(x, y, x2, y2);
      ellipse(x, y, 4, 4);
    }
  }

  private void drawGoal() {
    Translation2d g = planner.getFieldPlanner().getGoal();
    float gx = (float) g.getX() * scaleX;
    float gy = height - (float) g.getY() * scaleY;
    fill(255, 0, 0);
    noStroke();
    ellipse(gx, gy, 10, 10);
  }

  private void drawObstacles() {
    stroke(0);
    for (FieldPlanner.Obstacle obs : planner.getFieldPlanner().getObstacles()) {
      if (obs instanceof FieldPlanner.TeardropObstacle teardrop) {
        Translation2d loc = teardrop.loc;
        float x = (float) loc.getX() * scaleX;
        float y = height - (float) loc.getY() * scaleY;
        float radius = (float) teardrop.primaryRadius * scaleX;
        fill(255, 165, 0);
        ellipse(x, y, radius * 2, radius * 2);
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
  }

  private void drawVisionObstacles() {
    noStroke();
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
  }

  private void drawRobot() {
    fill(0, 255, 0, 80);
    noStroke();
    beginShape();
    for (Translation2d corner : robotCorners) {
      Translation2d rotated =
          corner.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
      float sx = (float) rotated.getX() * scaleX;
      float sy = height - (float) rotated.getY() * scaleY;
      vertex(sx, sy);
    }
    endShape(CLOSE);
  }
}
