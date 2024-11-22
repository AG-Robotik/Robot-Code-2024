package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.lib.math.InterpolatingTreeMap;
import frc.lib.math.InterpolatingDouble;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
  public static final String CANivoresCanBus = "CANivores";
  public static final double stickDeadband = 0.06;

  public static final class Limelight {
    public static final String kUpperName = "limelight-upper";
    public static final String kLowerName = "limelight-lower";
  }

  public static final class Led {
    public static int LedCount = 35;
    public static final int CANdleID = 26;
  }

  public static final class Intake {
    public static final int intakeID = 3;
    public static final int intakeAngleID = 4;
    public static final int intakeCanCoderID = 45;
    public static final int intakeSensorID = 8;

    public static final double absoluteMaxIntake = 116;
    public static final double absoluteMinIntake = 255;
    public static final double incrementalMinIntake = 47;
    public static final double incrementalMaxIntake = 7;
  }

  public static final class Feeder {
    public static final int feederID = 22;
    public static final int sensorID = 9;
  }

  public static final class Stick {
    public static final int stickID = 20;
    public static final double stowPos = 1;
    public static final double openPos = 183;
  }

  public static final class Angle {
    public static final int angleID = 25;
    public static final int angleCanCoderID = 44;

    public static final double hoodPIDMax = 0.9519;
    public static final double hoodPIDMin = -0.9519;
    public static final double hoodMaxAngle = 175.25;
    public static final double hoodMinAngle = 149.5;

    public static final double hoodKp = 0.4;
    public static final double hoodKi = 0;
    public static final double hoodKd = 0;
  }

  public static final class Shooter {
    public static final int shooterUID = 20;
    public static final int shooterDID = 21;
    public static final int stickID = 26;

    public static final Port sensorPort = I2C.Port.kOnboard;
    public static final int sensorDistance = 11;
  }

  public static final class Shoot {
    public static double[][] kShooterRPSValues = {
        { 44.47, 45 },
        { 13.21, 90 }
    };

    public static double[][] kHoodValues = {
        { 44.47, 175.00 },
        { 35.95, 173.84 },
        { 29.30, 168.48 },
        { 24.67, 164.70 },
        { 20.85, 161.36 },
        { 18.11, 159.25 },
        { 16.03, 157.06 },
        { 13.21, 155.00 }
    };

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRPSMap = new InterpolatingTreeMap<>();
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodMap = new InterpolatingTreeMap<>();
    static {
      for (double[] pair : kShooterRPSValues) {
        kRPSMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
      }
      for (double[] pair : kHoodValues) {
        kHoodMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
      }
    }
  }

  public static final class Climb {
    public static final int climbLeftID = 23;
    public static final int climbRightID = 24;
  }

  public static final class Swerve {
    public static final class AlingPids {
      public static final double rotKp = 0.02;
      public static final double rotKi = 0;
      public static final double rotKd = 0.001;
      public static final double rotPIDMin = -0.75;
      public static final double rotPIDMax = 0.75;

      public static final double SnapKp = 0.03;
      public static final double SnapKi = 0;
      public static final double SnapKd = 0;
      public static final double SnapPIDMin = -0.75;
      public static final double SnapPIDMax = 0.75;

      public static final double AlingXKp = 0.03;
      public static final double AlingXKi = 0;
      public static final double AlingXKd = 0;
      public static final double AlingXPIDMin = -0.2;
      public static final double AlingXPIDMax = 0.2;

      public static final double AlingYKp = 0.01;
      public static final double AlingYKi = 0;
      public static final double AlingYKd = 0;
      public static final double AlignYPIDMin = -0.3;
      public static final double AlignYPIDMax = 0.3;
    }

    public static final int pigeonID = 30;

    public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i
        .Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

    /* Drivetrain Constants */
    // public static final double trackWidth = Units.inchesToMeters(21.73);
    // public static final double wheelBase = Units.inchesToMeters(21.73);
    public static final double trackWidth = 0.595; // TODO: This must be tuned to specific robot
    public static final double wheelBase = 0.595; // TODO: This must be tuned to specific robot
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 35;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double openLoopRamp = 0.05;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12; // 0.05
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.32;
    public static final double driveKV = 1.51;
    public static final double driveKA = 0.27;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 6;
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0;

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 40;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(150.644 - 180); // 153.9 //150.08 //150.11
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 14;
      public static final int angleMotorID = 15;
      public static final int canCoderID = 42;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-38.408 - 180); // -39.1 // -38.3 // -38.58
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 13;
      public static final int canCoderID = 41;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(9.22 - 180); // 7.99 8.96 //9.4
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 16;
      public static final int angleMotorID = 17;
      public static final int canCoderID = 43;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(92.988 - 180); // 93.16 92.98 // 92.81
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
