// Gece saat 23.49, kod yazıyoruz ve kafalar pırıl pırıl... -Dünyanın en iyi iki yazılımcısı. (Biz ikimiz tek, tüm dünyaya denk!)
// Berk'imizi de asla unutmayız biz üçümüz tüm kainata tek atarız ;))
// Berk, 18 olunca gel knks ;) -Ahmet

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.otoCmd;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  public static Swerve m_Swerve = new Swerve();
  public static Superstructure m_Superstructure = new Superstructure();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  PathPlannerPath path_1 = PathPlannerPath.fromPathFile("6Node-Blue3-1");
  PathPlannerPath path_2 = PathPlannerPath.fromPathFile("6Node-Blue3-2");
  PathPlannerPath path_3 = PathPlannerPath.fromPathFile("6Node-Blue3-3");
  PathPlannerPath path_4 = PathPlannerPath.fromPathFile("6Node-Blue3-4");
  PathPlannerPath path_5 = PathPlannerPath.fromPathFile("5Node- 5");
  PathPlannerPath dpath_5 = PathPlannerPath.fromPathFile("6Node-Blue3-5");
  PathPlannerPath path_6 = PathPlannerPath.fromPathFile("5Node- 6");
  PathPlannerPath dpath_6 = PathPlannerPath.fromPathFile("6Node-Blue3-6");
  PathPlannerPath path_7 = PathPlannerPath.fromPathFile("6Node-Blue3-7");

  PathPlannerPath yeni_1 = PathPlannerPath.fromPathFile("Yeni- 1");
  PathPlannerPath yeni_2 = PathPlannerPath.fromPathFile("Yeni- 2");
  PathPlannerPath yeni_3 = PathPlannerPath.fromPathFile("Yeni- 3");
  PathPlannerPath yeni_4 = PathPlannerPath.fromPathFile("Yeni- 4");
  PathPlannerPath yeni_5 = PathPlannerPath.fromPathFile("Yeni- 5");

  PathPlannerPath berk_1 = PathPlannerPath.fromPathFile("5node-blue1-1");
  PathPlannerPath berk_2 = PathPlannerPath.fromPathFile("5node-blue1-2");
  PathPlannerPath berk_3 = PathPlannerPath.fromPathFile("5node-blue1-3");
  PathPlannerPath berk_4 = PathPlannerPath.fromPathFile("5node-blue1-4");

  public Command getAutonomousCommand() {
    if (Robot.m_autoSelected == "Mavi Bes") {
      m_Swerve.resetOdometry(path_1.getPreviewStartingHolonomicPose());
      return new otoCmd(false, true, 1.25)
          .andThen(AutoBuilder.followPath(path_1).raceWith(new otoCmd(true, false,
              777.81)))
          .andThen(new otoCmd(false, true, 1))
          .andThen(AutoBuilder.followPath(path_2).raceWith(new otoCmd(true, false,
              777.19)))
          .andThen(new otoCmd(false, true, 1))
          .andThen(AutoBuilder.followPath(path_3).raceWith(new otoCmd(true, false,
              777.08)))
          .andThen(new otoCmd(false, true, 1))
          .andThen(AutoBuilder.followPath(path_4).raceWith(new otoCmd(true, false,
              777.08)))
          .andThen(AutoBuilder.followPath(path_5).raceWith(new otoCmd(false, false,
              777.08)))
          .andThen(new otoCmd(false, true, 1))
          .andThen(AutoBuilder.followPath(path_6).raceWith(new otoCmd(false, false,
              777.08)));
    } else if (Robot.m_autoSelected == "Mavi Bes Berk") {
      m_Swerve.resetOdometry(berk_1.getPreviewStartingHolonomicPose());
      return new otoCmd(false, true, 1)
          .andThen(AutoBuilder.followPath(berk_1).raceWith(new otoCmd(true, false, 777.6)))
          .andThen(new otoCmd(false, true, 1))
          .andThen(AutoBuilder.followPath(berk_2).raceWith(new otoCmd(true, false, 777.6)))
          .andThen(new otoCmd(false, true, 1))
          .andThen(AutoBuilder.followPath(berk_3).raceWith(new otoCmd(true, false, 777.6)))
          .andThen(new otoCmd(false, true, 1))
          .andThen(AutoBuilder.followPath(berk_4).raceWith(new otoCmd(true, false, 777.6)))
          .andThen(new otoCmd(false, true, 1));
    } else {
      return null;
    }

    // m_Swerve.resetOdometry(path_1.getPreviewStartingHolonomicPose());
    // return AutoBuilder.followPath(path_1);
    // return new otoCmd(false, true, 1.25)
    // .andThen(AutoBuilder.followPath(path_1).raceWith(new otoCmd(true, false,
    // 777.81)))
    // .andThen(new otoCmd(false, true, 1))
    // .andThen(AutoBuilder.followPath(path_2).raceWith(new otoCmd(true, false,
    // 777.19)))
    // .andThen(new otoCmd(false, true, 1))
    // .andThen(AutoBuilder.followPath(path_3).raceWith(new otoCmd(true, false,
    // 777.08)))
    // .andThen(new otoCmd(false, true, 1))
    // .andThen(AutoBuilder.followPath(path_4).raceWith(new otoCmd(true, false,
    // 777.08)))
    // .andThen(AutoBuilder.followPath(dpath_5).raceWith(new otoCmd(false, false,
    // 777.08)))
    // .andThen(new otoCmd(false, true, 1))
    // .andThen(AutoBuilder.followPath(dpath_6).raceWith(new otoCmd(true, false,
    // 777.08)))
    // .andThen(AutoBuilder.followPath(path_7).raceWith(new otoCmd(false, false,
    // 777.08)))
    // .andThen(new otoCmd(false, true, 1)).andThen(new otoCmd(false, false, 1));
  }

}
