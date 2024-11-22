package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.math.Conversions;
import frc.lib.util.LimelightHelpers;

public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  Command m_autonomousCommand;
  RobotContainer m_robotContainer;

  Joystick stick_0 = new Joystick(0);
  Joystick stick_1 = new Joystick(1);

  double swerveSpeed = 1;
  double swerveAngularSpeed = 1;

  double snapvalue = 0;
  double AlignYvalue = 0;
  double AlignXvalue = 0;
  
  public static boolean amfLock = false;
  public static boolean sourceLock = false;
  public static boolean x;
  public static boolean manualStick = false;
  public static boolean manuelIntake = false;

  boolean climbMode = false;

  private static final String kDefaultAuto = "Bos";
  private static final String kMaviBes = "Mavi Bes";
  private static final String kMaviBerk = "Mavi Bes Berk";
  private static final String kBahadir = "Bahadir";
  static String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    RobotContainer.m_Swerve.forceZeroGyro();

    m_chooser.setDefaultOption("Bos", kDefaultAuto);
    m_chooser.addOption("Mavi Bes", kMaviBes);
    m_chooser.addOption("Mavi Bes Berk", kMaviBerk);
    m_chooser.addOption("Bahadir", kBahadir);
    SmartDashboard.putData("Otonom Secici", m_chooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("sensorFeeder", RobotContainer.m_Superstructure.getSensorFeeder());
    SmartDashboard.putBoolean("sensorIntake", RobotContainer.m_Superstructure.getSensorIntake());
    SmartDashboard.putNumber("Renk", RobotContainer.m_Superstructure.getSensorShooter());

    SmartDashboard.putNumber("upTemp", RobotContainer.m_Superstructure.upMotor.getDeviceTemp().getValue());
    SmartDashboard.putNumber("downTemp", RobotContainer.m_Superstructure.downMotor.getDeviceTemp().getValue());
    SmartDashboard.putNumber("RPS", RobotContainer.m_Superstructure.getShooterRPS());

    SmartDashboard.putNumber("tY", LimelightHelpers.getTY(Constants.Limelight.kLowerName));
    SmartDashboard.putNumber("ACI", RobotContainer.m_Superstructure.getHoodAngle());
    m_autoSelected = m_chooser.getSelected();
    System.out.println(m_autoSelected);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    // RobotContainer.m_Superstructure.setLarson(255, 0, 0, 2, 0.4, 1);
    RobotContainer.m_Superstructure.setLarson(255, 0, 0, 3, 0.4, 1);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double translationSup = stick_0.getRawAxis(1);
    double strafeSup = stick_0.getRawAxis(0);
    double rotationSup = stick_0.getRawAxis(2);

    double translationVal = -MathUtil.applyDeadband(translationSup * swerveSpeed, Constants.stickDeadband)
        + MathUtil.applyDeadband(AlignXvalue, 0.06);
    double strafeVal = -MathUtil.applyDeadband(strafeSup * swerveSpeed, Constants.stickDeadband);
    double rotationVal = -MathUtil.applyDeadband(rotationSup * swerveAngularSpeed, Constants.stickDeadband)
        + RobotContainer.m_Superstructure.rotPIDValue + RobotContainer.m_Superstructure.SnapPIDValue
        + MathUtil.applyDeadband(snapvalue, 0.06);

    RobotContainer.m_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity, true, true);

    // Gyro Sıfırlama
    if (stick_0.getRawButton(10)) {
      RobotContainer.m_Superstructure.zeroGyro();
      RobotContainer.m_Swerve.forceZeroGyro();
    }

    // Speaker Mod Aktif
    if (!sourceLock && !amfLock && !stick_0.getRawButton(1)) {
      RobotContainer.m_Superstructure.setSystem(stick_0.getRawButton(8), RobotContainer.m_Superstructure
          .checkShootingStatus(
              (RobotContainer.m_Superstructure.getSensorFeeder() || RobotContainer.m_Superstructure.getSensorIntake())
                  && stick_0.getRawButton(6)));
    }

    // Kaynak Mod Aktif
    if (stick_0.getRawButton(7)) {
      sourceLock = true;
    }

    // Amfi Mod Aktif
    if (stick_0.getRawButton(5) && RobotContainer.m_Superstructure.getSensorShooter() >= Constants.Shooter.sensorDistance) {
      amfLock = true;
    }

    // Amfi ve Kaynak Modunu kapatma
    if (stick_0.getRawButton(8) || stick_0.getRawButton(6)) {
      sourceLock = false;
      amfLock = false;
    }

    // Tüm sistemi kapat merkezine dön
    if (stick_0.getRawButton(4)) {
      RobotContainer.m_Superstructure.intakeLock = false;
      sourceLock = false;
      amfLock = false;
    }

    // Tüm sistemi kapat ve içindeki objeyi çıkar
    if (stick_0.getRawButton(1)) {
      RobotContainer.m_Superstructure.intakeMotor.set(-1);
      RobotContainer.m_Superstructure.feederMotor.set(-1);
      sourceLock = false;
      amfLock = false;
      RobotContainer.m_Superstructure.intakeLock = false;
      RobotContainer.m_Superstructure.setIntakePosition(1);
    }

    // Tırmanma Sistemi
    if (stick_1.getRawButton(4)) {
      RobotContainer.m_Superstructure.setClimbPosition(56000);
    } else if (stick_1.getRawButton(2)) {
      RobotContainer.m_Superstructure.setClimbSpeed(-0.25);
    } else {
      RobotContainer.m_Superstructure.setClimbStop();
    }

    // Manuel Intake
    if (stick_0.getPOV() == 0 && stick_0.getPOV() == 180) {
      manuelIntake = true;
    } else if (stick_0.getPOV() == 90 && stick_0.getPOV() == 270) {
      manuelIntake = false;
    }

    if (manuelIntake == true) {
      if (stick_1.getRawButton(1)) {
        RobotContainer.m_Superstructure.intakeLock = true;
      } else if (stick_1.getRawButton(3)) {
        RobotContainer.m_Superstructure.intakeLock = false;
      }
    }

    if (amfLock == true) {
      manualStick = false;
      RobotContainer.m_Superstructure.blink(0, 158, 186, 10);

      RobotContainer.m_Superstructure.setStickPosition(Constants.Stick.openPos);
      RobotContainer.m_Superstructure.setHoodAngle(164);

      var alliance = DriverStation.getAlliance();
      if (alliance.get() == Alliance.Blue) {
        snapvalue = RobotContainer.m_Superstructure.SnapPID(true, -90);
      } else {
        snapvalue = RobotContainer.m_Superstructure.SnapPID(true, 90);
      }

      if (LimelightHelpers.getTV(Constants.Limelight.kUpperName)
          && !Conversions.epsilonEquals(LimelightHelpers.getTX(Constants.Limelight.kUpperName), 0, 0.2)) {
        if (alliance.get() == Alliance.Blue) {
          AlignXvalue = -RobotContainer.m_Superstructure.AlingXPID(true, 0);
        } else {
          AlignXvalue = RobotContainer.m_Superstructure.AlingXPID(true, 0);
        }
      } else {
        AlignXvalue = 0;
      }

      if (LimelightHelpers.getTY(Constants.Limelight.kUpperName) >= 4) {
        x = true;
      }

      if (x) {
        if (RobotContainer.m_Superstructure.getHoodAngle() > 163) {
          RobotContainer.m_Superstructure.setShooterVelocityAmf(0, 70);
          if (RobotContainer.m_Superstructure.downMotor.getVelocity().getValue() > 65) {
            RobotContainer.m_Superstructure.setFeederVelocity(61);
            if (!RobotContainer.m_Superstructure.getSensorFeeder()) {
              amfLock = false;
            }
          }
        }
      }

    } else {
      RobotContainer.m_Superstructure.setStickPosition(0);
      snapvalue = 0;
      AlignXvalue = 0;
      x = false;
      if (manualStick) {
        RobotContainer.m_Superstructure.setStickPosition(Constants.Stick.openPos);
      } else {
        RobotContainer.m_Superstructure.setStickPosition(Constants.Stick.stowPos);
      }
    }

    // Manuel Stick
    if (stick_1.getRawButton(5)) {
      manualStick = true;
    } else {
      manualStick = false;
    }

    if (sourceLock == true) {
      RobotContainer.m_Superstructure.blink(255, 50, 0, 10);
      var alliance = DriverStation.getAlliance();
      if (alliance.get() == Alliance.Blue) {
        snapvalue = RobotContainer.m_Superstructure.SnapPID(true, 120);
      } else {
        snapvalue = RobotContainer.m_Superstructure.SnapPID(true, -120);
      }
      RobotContainer.m_Superstructure.setShooterVelocity(-10);
      if (RobotContainer.m_Superstructure.getSensorIntake()) {
        sourceLock = false;
      } else {
        RobotContainer.m_Superstructure.setFeederVelocity(-8);
        RobotContainer.m_Superstructure.setHoodAngle(164);
      }
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // RobotContainer.m_Superstructure.setStickPosition(20);
    if (stick_0.getRawButton(5) && stick_0.getRawButton(6) && stick_0.getRawButton(7) && stick_0.getRawButton(8)) {
      climbMode = true;
    }

    // if (stick_0.getRawButton(3)) {
    // RobotContainer.m_Superstructure.setHoodAngle(175.25);
    // } else if (stick_0.getRawButton(4)) {
    // RobotContainer.m_Superstructure.setHoodAngle(149.5);
    // } else {
    // RobotContainer.m_Superstructure.hoodAngleMotor.stopMotor();
    // }

    // if (stick_0.getRawButton(1)) {
    // RobotContainer.m_Superstructure.setClimbPosition(56000);
    // } else if (stick_0.getRawButton(2)) {
    // RobotContainer.m_Superstructure.climbLMotor.set(-0.25);
    // RobotContainer.m_Superstructure.climbRMotor.set(-0.25);
    // } else {
    // RobotContainer.m_Superstructure.climbLMotor.stopMotor();
    // RobotContainer.m_Superstructure.climbRMotor.stopMotor();
    // }

    if (!climbMode) {
      RobotContainer.m_Superstructure.setLarson(255, 255, 255, 2, 0.4, 1);

      double translationSup = stick_0.getRawAxis(1);
      double strafeSup = stick_0.getRawAxis(0);
      double rotationSup = stick_0.getRawAxis(2);

      double translationVal = -MathUtil.applyDeadband(translationSup * swerveSpeed,
          Constants.stickDeadband);
      double strafeVal = -MathUtil.applyDeadband(strafeSup * swerveSpeed,
          Constants.stickDeadband);
      double rotationVal = -MathUtil.applyDeadband(rotationSup *
          swerveAngularSpeed, Constants.stickDeadband)
          + RobotContainer.m_Superstructure.rotPIDValue;

      RobotContainer.m_Swerve.drive(
          new Translation2d(translationVal,
              strafeVal).times(Constants.Swerve.maxSpeed),
          rotationVal * Constants.Swerve.maxAngularVelocity,
          true,
          true);

      // if (stick_0.getRawButton(6)) {
      // if (RobotContainer.m_Superstructure.getSensorFeeder()) {
      // RobotContainer.m_Superstructure.setShooterVelocityAmf(0, 70);
      // if (RobotContainer.m_Superstructure.downMotor.getVelocity().getValue() > 65)
      // {
      // RobotContainer.m_Superstructure.setFeederVelocity(61);
      // }
      // } else {
      // RobotContainer.m_Superstructure.setFeederVelocity(0);
      // RobotContainer.m_Superstructure.setShooterVelocity(0);
      // }
      // }
    } else {
      RobotContainer.m_Superstructure.setRainbow(1, 1);

      if (stick_0.getRawButton(4)) {
        if (!Conversions.epsilonEquals(RobotContainer.m_Swerve.getGyroYaw().getDegrees(), -90, 0.5)) {
          snapvalue = RobotContainer.m_Superstructure.SnapPID(true, -90);
        } else {
          snapvalue = 0;
          if (LimelightHelpers.getTV(Constants.Limelight.kUpperName)
              && !Conversions.epsilonEquals(LimelightHelpers.getTX(Constants.Limelight.kUpperName), 0, 0.3)) {
            AlignXvalue = -RobotContainer.m_Superstructure.AlingXPID(true, 0);
          } else {
            AlignXvalue = 0;
          }
        }

      } else {
        snapvalue = 0;
        AlignXvalue = 0;
      }

      if (stick_0.getRawButton(1)) {
        climbMode = false;
      }

      double translationSup = stick_0.getRawAxis(1);
      double strafeSup = stick_0.getRawAxis(0);
      double rotationSup = 0;

      RobotContainer.m_Superstructure.setHoodAngle(164);

      double translationVal = -MathUtil.applyDeadband(translationSup * swerveSpeed,
          Constants.stickDeadband);
      double strafeVal = -MathUtil.applyDeadband(strafeSup * swerveSpeed,
          Constants.stickDeadband)
          + AlignXvalue;
      double rotationVal = -MathUtil.applyDeadband(rotationSup *
          swerveAngularSpeed, Constants.stickDeadband)
          + snapvalue;

      RobotContainer.m_Swerve.drive(
          new Translation2d(translationVal,
              strafeVal).times(Constants.Swerve.maxSpeed),
          rotationVal * Constants.Swerve.maxAngularVelocity,
          false,
          true);
    }

    if (stick_0.getRawButton(10)) {
      RobotContainer.m_Swerve.forceZeroGyro();
      RobotContainer.m_Superstructure.zeroGyro();
    }

    // RobotContainer.m_Superstructure.setIntakePosition(48);

    // if (stick_0.getRawButton(2)) {
    // if (!RobotContainer.m_Superstructure.getSensorFeeder()) {
    // RobotContainer.m_Superstructure.setIntakeSpeed(1);
    // RobotContainer.m_Superstructure.setFeederVelocity(8);
    // } else if (RobotContainer.m_Superstructure.getSensorFeeder()) {
    // RobotContainer.m_Superstructure.setIntakeSpeed(0);
    // RobotContainer.m_Superstructure.setFeederVelocity(7);
    // if (RobotContainer.m_Superstructure.m_colorSensor.getIR() >= 13) {
    // RobotContainer.m_Superstructure.setFeederVelocity(0);
    // }
    // } else {
    // RobotContainer.m_Superstructure.setIntakeSpeed(0);
    // RobotContainer.m_Superstructure.setFeederVelocity(0);
    // }
    // }

    // RobotContainer.m_Superstructure.setShooterVelocity(RobotContainer.m_Superstructure.map(stick_0.getRawAxis(1),
    // -1, 1, -90, 90));
    // SmartDashboard.putNumber("hedef",
    // RobotContainer.m_Superstructure.map(stick_0.getRawAxis(1), -1, 1, -90, 90));

    // RobotContainer.m_Superstructure.hoodAngleMotor.set(stick_0.getRawAxis(1));
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
