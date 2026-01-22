package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
  private Voltage intakeAppliedVoltage = Volts.mutable(0.0);

  private final FlywheelSim intakeSim;

  public IntakeIOSim() {
    intakeSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);
  }

  @Override
  public void setIntakerTarget(Voltage target) {
    this.intakeAppliedVoltage = target;
  }

  @Override
  public void stop() {
    setIntakerTarget(Volts.of(0));
  }

  @Override
  public void updateInputs(IntakeInputs input) {
    input.intakeAngularVelocity.mut_replace(intakeSim.getAngularVelocity());
    input.intakeSetVoltage.mut_replace(intakeAppliedVoltage);

    // Periodic
    intakeSim.setInputVoltage(intakeAppliedVoltage.in(Volts));
    intakeSim.update(0.02);
  }
}
