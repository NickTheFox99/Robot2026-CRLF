package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
  private Voltage m_appliedIntakeVoltage = Volts.mutable(0.0);
  private Voltage m_appliedIntakeExtenderVoltage = Volts.mutable(0.0);

  private IntakeInputsAutoLogged logged = new IntakeInputsAutoLogged();

  // physical constants for intake extender (NOT ACCURATE)
  private static final double kArmGearRatio = 100.0;
  private static final double kArmLengthMeters = Units.inchesToMeters(30);
  private static final double kArmMassKg = 5.0;
  private static final double kMinvoltageRads = 0.0;
  private static final double kMaxVoltageRads = 0.0;
  private final double kArmMOI = 1.0 / 3.0 * kArmMassKg * Math.pow(kArmLengthMeters, 2);

  // gains for extender (NOT ACCURATE)
  private static final double kS = 0.1;
  private static final double kG = 0.5;
  private static final double kV = 3.0;

  // TODO fix this later
  // private Angle startingExtenderVoltage = Degrees(0.0);

  private final ProfiledPIDController controller =
      // FILLER VALUES NOT ACCURATE
      new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(1000, 361));

  private ArmFeedforward ff;
  private final FlywheelSim intakeSim;
  private final FlywheelSim intakeExtenderSim;

  public IntakeIOSim() {
    intakeSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);
    intakeExtenderSim =
        // FILLER VALUES NOT ACCURATE
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);

    ff = new ArmFeedforward(kS, kG, kV);
    Voltage startingExtenderVoltage = Volts.zero();
    controller.setGoal(startingExtenderVoltage.in(Volts));
    logged.intakeVoltage = Volts.mutable(0);
  }

  /** Updates the applied voltage to drive the arm towards the noted position */
  // TODO: we need to implement the conversion of volts to arm angle

  /**
   * Sets the applied voltage to the volts
   *
   * @param volts
   */
  private void runVolts(Voltage intakeExtenderVolts) {
    this.m_appliedIntakeExtenderVoltage = intakeExtenderVolts;
  }

  @Override
  public void setIntakeTarget(Voltage target) {
    this.m_appliedIntakeVoltage = target;
  }

  @Override
  public void setIntakeExtenderTarget(Voltage voltage) {
    controller.setGoal(voltage.in(Volts));
  }

  @Override
  public void stop() {
    // TODO: need to be cleaned up
    // Voltage currentVoltage = Radians.of(intakeExtenderSim.setVoltage(0));
    // controller.reset(currentVoltage.in(Volts));
    setIntakeTarget(Volts.of(0));
    setIntakeExtenderTarget(Volts.of(0));
  }

  @Override
  public void updateInputs(IntakeInputs input) {
    // update inputs

    //  - intake
    input.intakeVoltage.mut_replace(m_appliedIntakeVoltage);
    input.intakeSetVoltage.mut_replace(m_appliedIntakeVoltage);
    //  - intake extender
    input.intakeExtenderVoltage.mut_replace(m_appliedIntakeExtenderVoltage);
    input.intakeExtenderSetVoltage.mut_replace(m_appliedIntakeExtenderVoltage);
    input.intakeExtenderSupplyCurrent.mut_replace(intakeExtenderSim.getCurrentDrawAmps(), Amps);

    // Periodic
    intakeSim.setInputVoltage(m_appliedIntakeVoltage.in(Volts));
    intakeSim.update(0.02);

    intakeExtenderSim.setInputVoltage(m_appliedIntakeExtenderVoltage.in(Volts));
    intakeExtenderSim.update(0.02);
  }
}
