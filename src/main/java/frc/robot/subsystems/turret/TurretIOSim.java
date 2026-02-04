package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.Gains;

public class TurretIOSim implements TurretIO {
  private Voltage turretAppliedVoltage = Volts.mutable(0.0);

  private final FlywheelSim turretSim;

  public TurretIOSim() {
    turretSim =
        // TODO Turret may not utilize flywheel, keep an eye on
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);
  }

  @Override
  public void stop() {}

  @Override
  public void updateInputs(TurretInputs input) {
    input.turretAngularVelocity.mut_replace(turretSim.getAngularVelocity());
    // input.turretSetAngle.mut_replace(turretAppliedVoltage);
    // TODO fix ^ to angle not volts
    // Periodic
    turretSim.setInputVoltage(turretAppliedVoltage.in(Volts));
    turretSim.update(0.02);
  }

  @Override
  public void setTarget(double position) {
    // TODO implement set angle on the sim
  }

  @Override
  public void setGains(Gains gains) {}
}
