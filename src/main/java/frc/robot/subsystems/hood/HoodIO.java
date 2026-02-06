package frc.robot.subsystems.hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodInputs {
    public MutAngle hoodAngle;
    public MutAngle hoodSetAngle;
  }

  public void setHoodTarget(Angle target);

  public void stop();

  public void updateInputs(HoodInputs input);
}
