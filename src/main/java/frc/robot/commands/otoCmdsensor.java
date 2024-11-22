package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;

public class otoCmdsensor extends Command {
  Superstructure m_Superstructure = RobotContainer.m_Superstructure;
  boolean intakeStatus;
  boolean shootingStatus;
  double endTime;
  Timer endTimer = new Timer();

  public otoCmdsensor(boolean intakeStatus, boolean shootingStatus) {
    this.intakeStatus = intakeStatus;
    this.shootingStatus = shootingStatus;
    addRequirements(m_Superstructure);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (shootingStatus) {
      m_Superstructure.autoRot();
    }
    m_Superstructure.setSystem(intakeStatus, m_Superstructure.checkShootingStatus(shootingStatus));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if(shootingStatus){
      return !m_Superstructure.getSensorFeeder();
    }else if(intakeStatus){
      return m_Superstructure.getSensorFeeder();
    }else{
      return false;
    }
  }
}
