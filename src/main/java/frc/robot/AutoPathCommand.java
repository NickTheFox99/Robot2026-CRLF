package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class AutoPathCommand extends PathPlannerAuto {
  public String m_autoName;

  public AutoPathCommand(String autoName) {
    super(autoName);
    m_autoName = autoName;
  }
}
