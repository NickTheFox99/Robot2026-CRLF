package frc.robot.subsystems.turret;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretInputs {
    // inputs for turret
    public MutAngularVelocity turretAngularVelocity;
    public MutVoltage turretVoltage;
    public MutVoltage turretSetVoltage;
  }

  public void setTarget(double angle);

  public void stop();

  public void updateInputs(TurretInputs input);
}
