package frc.robot.state;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface MatchEvents {

  public Trigger isDisabledTrigger();

  public Trigger isAutonomousEnableTrigger();

  public Trigger isEnabledTrigger();

  public Trigger isTeleopEnabledTrigger();
}
