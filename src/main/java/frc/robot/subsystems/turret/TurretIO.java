package frc.robot.subsystems.turret;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretInputs {
    // inputs for turret
    public MutAngularVelocity turretAngularVelocity;
    public MutAngle turretAngle;
    public MutAngle turretSetAngle;
    public MutAngle canCoderAngle1;
    public MutAngle canCoderAngle2;
  }

  public default void setTarget(double angle) {}
  ;

  public default void stop() {}
  ;

  public default void updateInputs(TurretInputs input) {}
  ;

  public default void setGains(Gains gains) {}
  ;
}
