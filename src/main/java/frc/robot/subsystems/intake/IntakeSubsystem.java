// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import org.littletonrobotics.junction.Logger;

/** Sets the controless the intake and endexer */
public class IntakeSubsystem extends SubsystemBase implements IntakeEvents {
  /** Creates a new ExampleSubsystem. */
  private IntakeIO m_IO;

  private final EnumState<IntakeState> currentGoal =
      new EnumState<>("Intake/States", IntakeState.IDLE);

  private IntakeInputsAutoLogged logged = new IntakeInputsAutoLogged();

  public IntakeSubsystem(IntakeIO IO) {
    m_IO = IO;
    logged.intakeVoltage = Volts.mutable(0);
    logged.intakeSetVoltage = Volts.mutable(0);
    logged.intakeSupplyCurrent = Amps.mutable(0);
    logged.intakeExtenderVoltage = Volts.mutable(0);
    logged.intakeExtenderSetVoltage = Volts.mutable(0);
    logged.intakeExtenderSupplyCurrent = Amps.mutable(0);
  }

  /**
   * Sets the speed for the intake
   *
   * @param speed
   */
  public void setIntakeSpeed(Voltage speed) {
    m_IO.setIntakeTarget(speed);
  }

  /**
   * Sets the speed for the intake
   *
   * @param speed
   */
  public void setIntakeVoltage(Voltage voltage) {
    m_IO.setIntakeExtenderTarget(voltage);
  }

  public Command intakeCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.INTAKING);
        });
  }

  public Command outtakeCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.OUTTAKING);
        });
  }

  public Command idleCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.IDLE);
        });
  }

  public void stop() {
    m_IO.stop();
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Intake", logged);
    switch (currentGoal.get()) {
      case INTAKING:
        // filler units rn
        m_IO.setIntakeTarget(Volts.of(11.0));
        m_IO.setIntakeExtenderTarget(Volts.of(5.0));
        break;
      case OUTTAKING:
        // filler units rn
        m_IO.setIntakeTarget(Volts.of(-11.0));
        m_IO.setIntakeExtenderTarget(Volts.of(-5.0));
        break;
      case IDLE:
        stop();
        break;
    }
  }

  @Override
  public Trigger isIdleTrigger() {
    return currentGoal.is(IntakeState.IDLE);
  }

  @Override
  public Trigger isIntakingTrigger() {
    return currentGoal.is(IntakeState.INTAKING);
  }

  @Override
  public Trigger isOuttakingTrigger() {
    return currentGoal.is(IntakeState.OUTTAKING);
  }
}
