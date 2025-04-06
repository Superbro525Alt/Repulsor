package com.vendor.jni.Mechanism;

import com.vendor.jni.Repulsor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;

public class Mechanism {
  public static class RepulsorMechanismCommand {
    Command m_command;
    Trigger m_trigger;

    public RepulsorMechanismCommand(Command command, Trigger trigger) {
      m_command = command;
      m_trigger = trigger;
    }

    public Command getCommand() {
      return m_command;
    }

    public Trigger getTrigger() {
      return m_trigger;
    }
  }

  public abstract static class RepulsorMechanism {
    public abstract List<RepulsorMechanismCommand> getCommands(Repulsor repulsor);
  }
}
