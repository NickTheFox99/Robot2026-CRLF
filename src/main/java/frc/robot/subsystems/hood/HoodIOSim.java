package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {

  private Angle hoodAngle = Degrees.mutable(0);

  private final SingleJointedArmSim hoodSim;

  // physical constants for hood (NOT ACCURATE)
  private static final DCMotor kArmMotor = DCMotor.getKrakenX60(1); // e.g., one NEO motor
  private static final double kGearing = 50.0; // e.g., 50:1 gear ratio
  private static final double kMoI = 1.5; // Moment of inertia in kg/m^2 (from CAD)
  private static final double kArmLength = Units.inchesToMeters(30.0); // e.g., 30 inches long
  private static final double kMinAngle = Units.degreesToRadians(-90.0); // e.g., -90 degrees
  private static final double kMaxAngle = Units.degreesToRadians(90.0); // e.g., 90 degrees
  private static final boolean kSimulateGravity = true;

  public HoodIOSim() {
    hoodSim =
        new SingleJointedArmSim(
            kArmMotor, kGearing, kMoI, kArmLength, kMinAngle, kMaxAngle, kSimulateGravity, 0);
  }

  @Override
  public void setHoodTarget(Angle angle) {
    this.hoodAngle = angle;
  }

  @Override
  public void stop() {
    this.hoodAngle = Degrees.mutable(0);
  }

  public void updateInputs(HoodInputs inputs) {
    inputs.hoodAngle.mut_replace(hoodSim.getAngleRads(), Radians);
    inputs.hoodSetAngle.mut_replace(hoodAngle);

    // Periodic
    hoodSim.setInput(hoodAngle.in(Degrees));
    hoodSim.update(0.02);
  }
}
