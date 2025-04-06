package com.vendor.jni.Mechanism;

import static edu.wpi.first.units.Units.*;

import com.vendor.jni.Mechanism.Mechanism.RepulsorMechanism;
import com.vendor.jni.Mechanism.Mechanism.RepulsorMechanismCommand;
import com.vendor.jni.Repulsor;
import com.vendor.jni.Setpoints.SetpointType;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;

public abstract class Intake extends RepulsorMechanism {
  private Elevator m_elevator;

  public Intake(Elevator e) {
    m_elevator = e;
  }

  public abstract Command intake();

  public abstract Command outtake();

  @Override
  public List<RepulsorMechanismCommand> getCommands(Repulsor repulsor) {
    return List.of(
        new RepulsorMechanismCommand(
            intake(),
            repulsor
                .within(Meters.of(0.05), SetpointType.kHumanPlayer)
                .and(() -> m_elevator.isStable())),
        new RepulsorMechanismCommand(
            outtake(),
            repulsor
                .within(Meters.of(0.05), SetpointType.kScore)
                .and(() -> m_elevator.isStable())));
  }
}
