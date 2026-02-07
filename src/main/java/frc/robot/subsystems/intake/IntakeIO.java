package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public MutVoltage intakeVoltage;
    public MutVoltage intakeSetVoltage;
    public MutCurrent intakeSupplyCurrent;

    public MutVoltage intakeExtenderVoltage;
    public MutVoltage intakeExtenderSetVoltage;
    public MutCurrent intakeExtenderSupplyCurrent;
    public MutAngle intakeExtenderAngle;
  }

  public default void setIntakeTarget(Voltage target) {}
  ;

  public default void setIntakeExtenderTarget(Voltage voltage) {}
  ;

  public default void stop() {}
  ;

  public default void updateInputs(IntakeInputs input) {}
  ;
}
