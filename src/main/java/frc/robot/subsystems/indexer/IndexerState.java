package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public enum IndexerState {
  IDLE(Volts.zero(), Volts.zero()),
  FEEDING(Volts.of(4.0), Volts.of(9.0));

  private Voltage m_indexerVolts;
  private Voltage m_feederVolts;

  private IndexerState(Voltage indexerVolts, Voltage feederVolts) {
    m_indexerVolts = indexerVolts;
    m_feederVolts = feederVolts;
  }

  public Voltage indexerVolts() {
    return m_indexerVolts;
  }

  public Voltage feederVolts() {
    return m_feederVolts;
  }
}
