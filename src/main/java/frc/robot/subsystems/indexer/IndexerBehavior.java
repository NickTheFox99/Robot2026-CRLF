package frc.robot.subsystems.indexer;

import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

public class IndexerBehavior extends SubsystemBehavior {
  private final IndexerSubsystem indexer;

  public IndexerBehavior(IndexerSubsystem indexer) {
    this.indexer = indexer;
  }

  @Override
  public void configure(AllEvents events) {
    events.goals().isIdleTrigger().whileTrue(indexer.idleCommand());
    events
        .goals()
        .isShootingTrigger()
        .or(events.goals().isPassingTrigger())
        .whileTrue(indexer.indexingCommand());
    events.goals().isIntakingTrigger().whileTrue(indexer.indexingCommand());
    events.goals().isOuttakingTrigger().whileTrue(indexer.idleCommand());
    events.goals().isClimbingL0().whileTrue(indexer.idleCommand());
    events.goals().isClimbingL1().whileTrue(indexer.idleCommand());
    events.goals().isClimbingL2().whileTrue(indexer.idleCommand());
    events.goals().isClimbingL3().whileTrue(indexer.idleCommand());
  }
}
