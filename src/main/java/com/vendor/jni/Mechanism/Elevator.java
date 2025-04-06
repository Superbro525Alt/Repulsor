package com.vendor.jni.Mechanism;

import static edu.wpi.first.units.Units.Meters;

import com.vendor.jni.Mechanism.Mechanism.RepulsorMechanism;
import com.vendor.jni.Mechanism.Mechanism.RepulsorMechanismCommand;
import com.vendor.jni.Repulsor;
import com.vendor.jni.Setpoints.HeightSetpoint;
import com.vendor.jni.Setpoints.SetpointType;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;

public abstract class Elevator extends RepulsorMechanism {

  public Elevator() {
    super();
  }

  /**
   * @return The fixed height from the ground to the elevator's lowest position.
   */
  public abstract Distance getBaseHeight();

  /**
   * @return The maximum vertical travel distance of the elevator.
   */
  public abstract Distance getMaxExtension();

  /** Converts a local height (relative to the elevator's base) into a motion command. */
  public abstract Command goToLocalHeight(Distance localHeight);

  /** Checks if the elevator is at the correct height */
  public abstract boolean isStable();

  /**
   * Move to a specific height from the ground (e.g. 1.1m for scoring). This works regardless of
   * elevator size, base height, or configuration.
   */
  public final Command goToSetpoint(Distance fieldHeight) {
    Distance base = getBaseHeight();
    Distance maxTravel = getMaxExtension();

    // Clamp fieldHeight to be within what the elevator can reach
    double min = base.in(Meters);
    double max = base.in(Meters) + maxTravel.in(Meters);
    double target = Math.max(min, Math.min(fieldHeight.in(Meters), max));

    Distance local = Meters.of(target - base.in(Meters));

    return goToLocalHeight(local);
  }

  /** Returns a command list used by the repulsor, configured to go to a common field setpoints. */
  @Override
  public List<RepulsorMechanismCommand> getCommands(Repulsor repulsor) {
    return List.of(
        new RepulsorMechanismCommand(
            goToSetpoint(Meters.of(0.18)), // Processor
            repulsor
                .within(Meters.of(0.05), SetpointType.kProcessor)
                .and(() -> repulsor.getTargetHeight() == HeightSetpoint.PROCESSOR)),
        new RepulsorMechanismCommand(
            goToSetpoint(Meters.of(0.46)), // L1
            repulsor
                .within(Meters.of(0.05), SetpointType.kScore)
                .and(() -> repulsor.getTargetHeight() == HeightSetpoint.L1)),
        new RepulsorMechanismCommand(
            goToSetpoint(Meters.of(0.75)), // Deep Cage
            repulsor
                .within(Meters.of(0.05), SetpointType.kDeepCage)
                .and(() -> repulsor.getTargetHeight() == HeightSetpoint.DEEP_CAGE)),
        new RepulsorMechanismCommand(
            goToSetpoint(Meters.of(0.81)), // L2
            repulsor
                .within(Meters.of(0.05), SetpointType.kScore)
                .and(() -> repulsor.getTargetHeight() == HeightSetpoint.L2)),
        new RepulsorMechanismCommand(
            goToSetpoint(Meters.of(0.95)), // Coral Station
            repulsor
                .within(Meters.of(0.05), SetpointType.kHumanPlayer)
                .and(() -> repulsor.getTargetHeight() == HeightSetpoint.CORAL_STATION)),
        new RepulsorMechanismCommand(
            goToSetpoint(Meters.of(1.21)), // L3
            repulsor
                .within(Meters.of(0.05), SetpointType.kScore)
                .and(() -> repulsor.getTargetHeight() == HeightSetpoint.L3)),
        new RepulsorMechanismCommand(
            goToSetpoint(Meters.of(1.83)), // L4
            repulsor
                .within(Meters.of(0.05), SetpointType.kScore)
                .and(() -> repulsor.getTargetHeight() == HeightSetpoint.L4)));
  }
}
