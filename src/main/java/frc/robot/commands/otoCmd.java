package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;

public class otoCmd extends Command {
  Superstructure m_Superstructure = RobotContainer.m_Superstructure;
  boolean intakeStatus;
  boolean shootingStatus;
  double endTime;
  Timer endTimer = new Timer();

  public otoCmd(boolean intakeStatus, boolean shootingStatus, double endTime) {
    this.intakeStatus = intakeStatus;
    this.shootingStatus = shootingStatus;
    this.endTime = endTime;
    addRequirements(m_Superstructure);
  }

  @Override
  public void initialize() {
    endTimer.reset();
  }

  @Override
  public void execute() {
    endTimer.start();
    if (shootingStatus) {
      m_Superstructure.autoRot();
    }
    m_Superstructure.setSystem(intakeStatus, m_Superstructure.checkShootingStatus(shootingStatus));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (endTimer.get() > endTime) {
      return true;
    } else {
      return false;
    }
  }
}
