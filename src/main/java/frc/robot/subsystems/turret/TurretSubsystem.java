package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase implements TurretEvents {

  private TurretIO m_IO;

  private Supplier<Pose2d> m_poseSupplier;

  private final EnumState<TurretState> currentGoal =
      new EnumState<>("Turret/States", TurretState.IDLE);

  private static final double VIEW_CHANGE = 0.0;
  private static final double TURRET_MIN_POS = -105.0; // -160.0;//137.0
  private static final double TURRET_MAX_POS = 105.0; // 110.0;//115.0

  private TurretInputsAutoLogged logged = new TurretInputsAutoLogged();

  /**
   * 
   * @param IO
   * @param poseSupplier passes in Drive::getAutoAlignPose
   */
  public TurretSubsystem(TurretIO IO, Supplier<Pose2d> poseSupplier) {
    m_IO = IO;
    m_poseSupplier = poseSupplier;
  }

  /**
   *
   *
   * <h3>setPosition</h3>
   *
   * Sets the target angle of the subsystem
   *
   * @param angle The angle in degrees from the horizontal
   */
  public void setPosition(double angle) {
    double clampedPosition = MathUtil.clamp(angle, TURRET_MIN_POS, TURRET_MAX_POS) + VIEW_CHANGE;
    Logger.recordOutput(this.getClass().getSimpleName() + "/ClampedPosition", clampedPosition);
    m_IO.setTarget(clampedPosition);
  }

  public Command idleCommand() {
    return runOnce(
        () -> {
          currentGoal.set(TurretState.IDLE);
        });
  }

  public Command aimingCommand() {
    return runOnce(
        () -> {
          currentGoal.set(TurretState.AIMING);
        });
  }

  public Command passingCommand() {
    return runOnce(
        () -> {
          currentGoal.set(TurretState.PASSING);
        });
  }

  @Override
  public void periodic() {
    Pose2d currentPose = m_poseSupplier.get();
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Turret", logged);
    switch (currentGoal.get()) {
      case AIMING:
        m_IO.setTarget(25);
        break;
      case PASSING:
        m_IO.setTarget(10);
        break;
      case IDLE:
        m_IO.setTarget(0);
        break;
    }
  }

  @Override
  public Trigger isIdleTrigger() {
    return currentGoal.is(TurretState.IDLE);
  }

  @Override
  public Trigger isPassingTrigger() {
    return currentGoal.is(TurretState.PASSING);
  }
}
