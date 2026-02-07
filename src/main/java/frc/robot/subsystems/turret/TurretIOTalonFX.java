package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.Gains;

public class TurretIOTalonFX implements TurretIO {

  public MotionMagicTorqueCurrentFOC request;

  public TalonFX motor;

  private Angle m_setAngle;

  // TODO initialize offset while disabled
  //      while disabled, get initial offset from the 2 external cancoders
  //      Use this offset when doing internal encoder position
  private Angle offset;

  private double encoder1ratio;
  private double encoder2ratio;

  private CANcoder canCoder1;

  private CANcoder canCoder2;

  public TurretIOTalonFX(int motorID, int canCoder1ID, int canCoder2ID, CANBus canbus) {
    motor = new TalonFX(motorID, canbus);
    canCoder1 = new CANcoder(canCoder1ID, canbus);
    canCoder2 = new CANcoder(canCoder2ID, canbus);
    m_setAngle = Degrees.of(0.0);
  }

  public void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    cfg.CurrentLimits.SupplyCurrentLimit = 80.0;
    cfg.CurrentLimits.StatorCurrentLimit = 80.0;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // TODO find actual gear ratios & set encoder ratios (math)
    cfg.Feedback.SensorToMechanismRatio = 1.0;
    cfg.Feedback.RotorToSensorRatio = 1.0;

    canCoder1.setPosition(Units.degreesToRotations(0.0));
    canCoder2.setPosition(Units.degreesToRotations(0.0));
  }

  @Override
  public void setTarget(double angle) {
    if (angle != m_setAngle.in(Degrees)) {
      request = request.withPosition(Degrees.of(angle)).withSlot(0);
      motor.setControl(request);
      m_setAngle = Degrees.of(angle);
    }
  }

  @Override
  public void stop() {}

  @Override
  public void updateInputs(TurretInputs inputs) {
    inputs.turretAngle.mut_replace(motor.getPosition().getValue());
    inputs.turretAngularVelocity.mut_replace(motor.getVelocity().getValue());
    inputs.turretSetAngle.mut_replace(m_setAngle);
    inputs.canCoderAngle1.mut_replace(canCoder1.getAbsolutePosition().getValue());
    inputs.canCoderAngle2.mut_replace(canCoder2.getAbsolutePosition().getValue());
  }

  public void setGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kG = gains.kG;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    motor.getConfigurator().apply(slot0Configs);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    motor.getConfigurator().apply(motionMagicConfigs);
  }
}
