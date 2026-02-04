package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.goals.RobotGoal;
import frc.robot.goals.RobotGoals;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoCommandManager {

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public AutoCommandManager(Drive drive, RobotGoals goals) {
    configureNamedCommands(drive, goals);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void configureNamedCommands(Drive drive, RobotGoals goals) {
    NamedCommands.registerCommand(
        "Shooting", goals.setGoalCommand(RobotGoal.SHOOTING).andThen(new WaitCommand(5.0)));
    NamedCommands.registerCommand("Intaking", goals.setGoalCommand(RobotGoal.INTAKING));
    // TODO: Make only the intake retract
    NamedCommands.registerCommand("RetractIntake", goals.setGoalCommand(RobotGoal.IDLE));
  }
}
