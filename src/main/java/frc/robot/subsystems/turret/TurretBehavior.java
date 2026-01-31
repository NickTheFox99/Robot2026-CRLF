package frc.robot.subsystems.turret;

import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

// TODO Later
public class TurretBehavior extends SubsystemBehavior {

  private final TurretSubsystem turret;

  public TurretBehavior(TurretSubsystem turret) {
    this.turret = turret;
  }

  @Override
  public void configure(AllEvents events) {
    events.match().isTeleopEnabled().whileTrue(turret.aimingCommand());
    events.goals().isPassingTrigger().whileTrue(turret.passingCommand());
    events.goals().isIdleTrigger().whileTrue(turret.idleCommand());
  }
}
