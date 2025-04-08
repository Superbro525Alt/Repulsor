package com.vendor.jni;

import com.vendor.jni.Fallback.PlannerFallback;
import com.vendor.jni.Mechanism.Mechanism.RepulsorMechanism;
import com.vendor.jni.Setpoints.HeightSetpoint;
import com.vendor.jni.Setpoints.RepulsorSetpoint;
import com.vendor.jni.Setpoints.SetpointType;
import com.vendor.jni.Setpoints.SetpointsReefscape;
import com.vendor.jni.Vision.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

public class Repulsor {

  public enum UsageType {
    kFullAuto,
    kAutoDrive
  }

  // ==========================
  // ROBOT CONFIG
  // ==========================
  private double robot_x;
  private double robot_y;
  private double algae_offset;
  private double coral_offset;

  // ==========================
  // PATHING
  // ==========================
  private FieldPlanner m_planner;
  private VisionPlanner m_visionPlanner = new VisionPlanner();
  private Drive m_drive;
  private UsageType m_usageType = UsageType.kAutoDrive;
  private RepulsorSetpoint m_currentGoal =
      new RepulsorSetpoint(SetpointsReefscape.A, HeightSetpoint.L2);

  // ===========================
  // FULL AUTO FUNCTIONS
  // ===========================
  private Optional<Trigger> m_collectedCondition;
  private Optional<Trigger> m_readyToCollectCondition;
  private RepulsorSetpoint m_nextScore =
      new RepulsorSetpoint(SetpointsReefscape.A, HeightSetpoint.L2);
  private SetpointsReefscape m_nextHp = SetpointsReefscape.RIGHT_HP;
  private List<RepulsorMechanism> m_mechanisms = new ArrayList<RepulsorMechanism>();

  public Repulsor(
      Drive drive,
      UsageType usageType,
      double robot_x,
      double robot_y,
      double coral_offset,
      double algae_offset) {
    this.m_drive = drive;
    this.m_usageType = usageType;
    this.robot_x = robot_x;
    this.robot_y = robot_y;
    this.coral_offset = coral_offset;
    this.algae_offset = algae_offset;

    m_planner =
        new FieldPlanner()
            .withFallbackPlan(
                new ExtraPathing()
                .new FallbackNextAlong(robot_x, robot_y, coral_offset, algae_offset));
  }

  public Repulsor(
      Drive drive, double robot_x, double robot_y, double coral_offset, double algae_offset) {
    this(drive, UsageType.kAutoDrive, robot_x, robot_y, coral_offset, algae_offset);
  }

  public Repulsor withCollectedCondition(Trigger collectedCondition) {
    m_collectedCondition = Optional.of(collectedCondition);
    return this;
  }

  public Repulsor withReadyToCollectCondition(Trigger readyToCollectCondition) {
    m_readyToCollectCondition = Optional.of(readyToCollectCondition);
    return this;
  }

  public Repulsor withInitialNext(RepulsorSetpoint setpoint) {
    if (setpoint.point() == SetpointsReefscape.RIGHT_HP
        || setpoint.point() == SetpointsReefscape.LEFT_HP) {
      throw new Error("Next score setpoint cannot be a human player one");
    }
    m_nextScore = setpoint;
    return this;
  }

  public Repulsor withInitialHP(RepulsorSetpoint setpoint) {
    if (setpoint.point() != SetpointsReefscape.RIGHT_HP
        && setpoint.point() != SetpointsReefscape.LEFT_HP) {
      throw new Error("Next hp setpoint cannot be a score one");
    }
    m_nextScore = setpoint;
    return this;
  }

  public void setNextScore(RepulsorSetpoint next) {
    if (next.point() == SetpointsReefscape.RIGHT_HP || next.point() == SetpointsReefscape.LEFT_HP) {
      throw new Error("Next score setpoint cannot be a human player one");
    }
    m_nextScore = next;
  }

  public void setHP(SetpointsReefscape next) {
    if (next != SetpointsReefscape.RIGHT_HP && next != SetpointsReefscape.LEFT_HP) {
      throw new Error("Next hp setpoint cannot be a score one");
    }
    m_nextHp = next;
  }

  public Repulsor withFallback(PlannerFallback fallback) {
    m_planner = m_planner.withFallback(fallback);
    return this;
  }

  public Repulsor withMechanism(RepulsorMechanism mechanism) {
    m_mechanisms.add(mechanism);
    return this;
  }

  public void addMechanism(RepulsorMechanism mechanism) {
    m_mechanisms.add(mechanism);
  }

  public Repulsor withVision(Vision vision) {
    m_visionPlanner.addVision(vision);
    return this;
  }

  public void addVision(Vision vision) {
    m_visionPlanner.addVision(vision);
  }

  public FieldPlanner getFieldPlanner() {
    return m_planner;
  }

  public VisionPlanner getVisionPlanner() {
    return m_visionPlanner;
  }

  public void setup() {
    if (m_usageType == UsageType.kAutoDrive
        || m_readyToCollectCondition.isEmpty()
        || m_collectedCondition.isEmpty()) {
      throw new Error("Not Setup Properly");
    }

    m_collectedCondition
        .get()
        .onTrue(
            Commands.defer(
                () -> alignTo(m_nextScore, m_readyToCollectCondition.get()), Set.of(m_drive)));
    m_readyToCollectCondition
        .get()
        .onTrue(
            Commands.defer(
                () ->
                    alignTo(
                        new RepulsorSetpoint(m_nextHp, HeightSetpoint.CORAL_STATION),
                        m_collectedCondition.get()),
                Set.of(m_drive)));

    m_mechanisms.forEach(
        (mech) -> {
          mech.getCommands(this)
              .forEach(
                  (c) -> {
                    c.getTrigger().onTrue(c.getCommand());
                  });
        });
  }

  public Command alignTo(Supplier<RepulsorSetpoint> point, Trigger until) {
    return Commands.run(
            () -> {
              m_currentGoal = point.get();
              m_planner.setGoal(
                  point.get().point().getPose(robot_x, robot_y, coral_offset, algae_offset));
              m_drive.runVelocity(
                  m_planner
                      .calculate(
                          m_drive.getPose(),
                          m_visionPlanner.getObstacles(),
                          robot_x,
                          robot_y,
                          true,
                          new ArrayList<Pose2d>())
                      .asChassisSpeeds(m_drive.getOmegaPID(), m_drive.getPose().getRotation()));
            },
            m_drive)
        .until(until);
  }

  public Command alignTo(RepulsorSetpoint point, Trigger until) {
    return Commands.run(
            () -> {
              m_currentGoal = point;
              m_planner.setGoal(
                  point.point().getPose(robot_x, robot_y, coral_offset, algae_offset));
              m_drive.runVelocity(
                  m_planner
                      .calculate(
                          m_drive.getPose(),
                          m_visionPlanner.getObstacles(),
                          robot_x,
                          robot_y,
                          true,
                          new ArrayList<Pose2d>())
                      .asChassisSpeeds(m_drive.getOmegaPID(), m_drive.getPose().getRotation()));
            },
            m_drive)
        .until(until);
  }

  public HeightSetpoint getTargetHeight() {
    return m_currentGoal.h();
  }

  public Trigger within(Distance d) {
    return new Trigger(
        () -> {
          Optional<Distance> err = m_planner.getErr();
          if (err.isEmpty()) {
            return false;
          }
          return err.get().lt(d);
        });
  }

  public Trigger within(Distance d, SetpointType t) {
    return new Trigger(
        () -> {
          Optional<Distance> err = m_planner.getErr();
          if (err.isEmpty()) {
            return false;
          }
          return err.get().lt(d) && m_currentGoal.point().type() == t;
        });
  }

  public Trigger within(Distance d, SetpointsReefscape p) {
    return new Trigger(
        () -> {
          Optional<Distance> err = m_planner.getErr();
          if (err.isEmpty()) {
            return false;
          }
          return err.get().lt(d) && m_currentGoal.point() == p;
        });
  }
}
