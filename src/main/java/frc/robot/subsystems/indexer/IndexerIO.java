package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public class IndexerInputs {
    public MutCurrent indexerSupplyCurrent;
    public MutVoltage indexerVoltage;
    public MutVoltage indexerSetVoltage;

    public MutCurrent feederSupplyCurrent;
    public MutVoltage feederVoltage;
    public MutVoltage feederSetVoltage;
  }

  public void setIndexerTarget(Voltage volts);

  public void setFeederTarget(Voltage volts);

  public void stop();

  public void updateInputs(IndexerInputs inputs);
}
