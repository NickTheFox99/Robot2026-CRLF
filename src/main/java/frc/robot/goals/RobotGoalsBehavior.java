package frc.robot.goals;

import frc.robot.operator.OperatorIntentEvents;
import frc.robot.util.GoalBehavior;

/**
 * Wires operator button presses to robot goal state.
 *
 * <p>This is the teleop-specific logic. Autonomous bypasses this and calls
 * RobotRobotGoals.getInstance().setGoal() directly.
 *
 * <p>TODO (students): Map your button intents to goals here Example:
 * intent.wantsToScore().onTrue(RobotGoals.getInstance().setGoal(RobotGoal.LAUNCHING))
 * .onFalse(RobotGoals.getInstance().setGoal(RobotGoal.IDLE));
 */
public class RobotGoalsBehavior extends GoalBehavior {

  public RobotGoalsBehavior() {}

  @Override
  public void configure(OperatorIntentEvents intent) {
    intent
        .wantsToScoreTrigger()
        .onTrue(RobotGoals.getInstance().setGoalCommand(RobotGoal.SHOOTING))
        .onFalse(RobotGoals.getInstance().setGoalCommand(RobotGoal.INTAKING));
    intent
        .wantsToOuttake()
        .onTrue(RobotGoals.getInstance().setGoalCommand(RobotGoal.OUTTAKING))
        .onFalse(RobotGoals.getInstance().setGoalCommand(RobotGoal.INTAKING));
    intent
        .wantsToPass()
        .onTrue(RobotGoals.getInstance().setGoalCommand(RobotGoal.PASSING))
        .onFalse(RobotGoals.getInstance().setGoalCommand(RobotGoal.INTAKING));
    intent
        .wantsToClimbL0()
        .onTrue(RobotGoals.getInstance().setGoalCommand(RobotGoal.CLIMBINGL0))
        .onFalse(RobotGoals.getInstance().setGoalCommand(RobotGoal.IDLE));
    intent
        .wantsToClimbL1()
        .onTrue(RobotGoals.getInstance().setGoalCommand(RobotGoal.CLIMBINGL1))
        .onFalse(RobotGoals.getInstance().setGoalCommand(RobotGoal.IDLE));

    intent
        .wantsToClimbL2()
        .onTrue(RobotGoals.getInstance().setGoalCommand(RobotGoal.CLIMBINGL2))
        .onFalse(RobotGoals.getInstance().setGoalCommand(RobotGoal.IDLE));

    intent
        .wantsToClimbL3()
        .onTrue(RobotGoals.getInstance().setGoalCommand(RobotGoal.CLIMBINGL3))
        .onFalse(RobotGoals.getInstance().setGoalCommand(RobotGoal.IDLE));
  }
}
