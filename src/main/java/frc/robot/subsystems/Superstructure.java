package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.lib.math.InterpolatingDouble;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Superstructure extends SubsystemBase {

  static CANdle mLed = new CANdle(Constants.Led.CANdleID);

  /////////////////////////////////////////////////////////////////////////////////////////////

  public CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.intakeID, MotorType.kBrushless);
  RelativeEncoder m_IntakeRollerEncoder;

  /////////////////////////////////////////////////////////////////////////////////////////////

  CANSparkMax intakeAngleMotor = new CANSparkMax(Constants.Intake.intakeAngleID, MotorType.kBrushless);

  DigitalInput sensorIntake = new DigitalInput(Constants.Intake.intakeSensorID);

  WPI_CANCoder intakeAngleEncoder = new WPI_CANCoder(Constants.Intake.intakeCanCoderID);
  CANCoderConfiguration _canCoderConfigurationIntake = new CANCoderConfiguration();

  RelativeEncoder m_IntakeEncoder;
  SparkPIDController m_IntakePidController;
  double kP, kI, kMaxOutput, kMinOutput;

  public double mappedAbsVal;

  /////////////////////////////////////////////////////////////////////////////////////////////

  Slot0Configs feederControllerConfig;
  VelocityVoltage m_feederVoltageVelocity = new VelocityVoltage(0);
  public TalonFX feederMotor = new TalonFX(Constants.Feeder.feederID);

  DigitalInput sensorInput = new DigitalInput(Constants.Feeder.sensorID);

  /////////////////////////////////////////////////////////////////////////////////////////////

  Slot0Configs shooterControllerConfig;
  VelocityTorqueCurrentFOC m_shooterVelocityTorque = new VelocityTorqueCurrentFOC(0);
  public TalonFX upMotor = new TalonFX(Constants.Shooter.shooterUID, Constants.CANivoresCanBus);
  public TalonFX downMotor = new TalonFX(Constants.Shooter.shooterDID, Constants.CANivoresCanBus);

  public ColorSensorV3 shooterSensor = new ColorSensorV3(Constants.Shooter.sensorPort);

  /////////////////////////////////////////////////////////////////////////////////////////////

  WPI_TalonFX stickMotor = new WPI_TalonFX(Constants.Shooter.stickID);

  /////////////////////////////////////////////////////////////////////////////////////////////

  public WPI_TalonFX hoodAngleMotor = new WPI_TalonFX(Constants.Angle.angleID);

  public WPI_CANCoder hoodAngleEncoder = new WPI_CANCoder(Constants.Angle.angleCanCoderID);
  CANCoderConfiguration _canCoderConfigurationAngle = new CANCoderConfiguration();

  PIDController hoodPID = new PIDController(Constants.Angle.hoodKp, Constants.Angle.hoodKi, Constants.Angle.hoodKd);

  /////////////////////////////////////////////////////////////////////////////////////////////

  public WPI_TalonFX climbLMotor = new WPI_TalonFX(Constants.Climb.climbLeftID);
  public WPI_TalonFX climbRMotor = new WPI_TalonFX(Constants.Climb.climbRightID);

  /////////////////////////////////////////////////////////////////////////////////////////////

  public boolean intakeLock = false;
  boolean prepareLock = false;
  boolean IsShooterReady;
  boolean IsThereAnyTarget;
  boolean IsRobotCentered;
  boolean IsHoodReady;
  boolean ready;
  int o;

  /////////////////////////////////////////////////////////////////////////////////////////////

  PIDController rotationPID = new PIDController(Constants.Swerve.AlingPids.rotKp, Constants.Swerve.AlingPids.rotKi,
      Constants.Swerve.AlingPids.rotKd);
  public double rotPIDValue = 0;
  public double rotPIDSetpoint = 0;
  public boolean rotPIDLock = false;

  /////////////////////////////////////////////////////////////////////////////////////////////

  PIDController SnapPID = new PIDController(Constants.Swerve.AlingPids.SnapKp, Constants.Swerve.AlingPids.SnapKi,
      Constants.Swerve.AlingPids.SnapKd);
  public double SnapPIDValue = 0;
  public double SnapPIDSetpoint = 0;
  boolean TargetSeenInTheFrame = false;

  /////////////////////////////////////////////////////////////////////////////////////////////

  PIDController AlingXPID = new PIDController(Constants.Swerve.AlingPids.AlingXKp, Constants.Swerve.AlingPids.AlingXKi,
      Constants.Swerve.AlingPids.AlingXKd);
  public double AlingXPIDValue = 0;
  public double AlingXPIDSetpoint = 0;

  /////////////////////////////////////////////////////////////////////////////////////////////

  PIDController AlignYPID = new PIDController(Constants.Swerve.AlingPids.AlingYKp, Constants.Swerve.AlingPids.AlingYKp,
      Constants.Swerve.AlingPids.AlingYKp);
  public double AlignYPIDValue = 0;
  public double AlignYPIDSetpoint = 0;

  /////////////////////////////////////////////////////////////////////////////////////////////

  public Superstructure() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    mLed.configAllSettings(configAll, 100);

    /////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////

    CameraServer.startAutomaticCapture().setResolution(360, 240);

    /////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////

    rotationPID.reset();
    hoodPID.reset();
    SnapPID.reset();
    AlignYPID.reset();
    AlingXPID.reset();

    /////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////

    intakeMotor.setSmartCurrentLimit(40);
    intakeMotor.setInverted(true);

    m_IntakeRollerEncoder = intakeMotor.getEncoder();

    /////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////

    intakeAngleMotor.restoreFactoryDefaults();
    intakeAngleMotor.setInverted(true);
    intakeAngleMotor.setIdleMode(IdleMode.kBrake);
    intakeAngleMotor.setSmartCurrentLimit(40);

    m_IntakeEncoder = intakeAngleMotor.getEncoder();
    m_IntakeEncoder.setPosition(0);
    m_IntakePidController = intakeAngleMotor.getPIDController();
    // PID coefficients
    kP = 0.00005;
    kI = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    m_IntakePidController.setP(kP);
    m_IntakePidController.setI(kI);
    m_IntakePidController.setOutputRange(kMinOutput, kMaxOutput);

    m_IntakePidController.setSmartMotionMaxVelocity(200000, 0);
    m_IntakePidController.setSmartMotionMinOutputVelocity(0, 0);
    m_IntakePidController.setSmartMotionMaxAccel(100000, 0);
    m_IntakePidController.setSmartMotionAllowedClosedLoopError(1, 0);

    intakeAngleEncoder.configAllSettings(_canCoderConfigurationIntake);
    intakeAngleEncoder.configMagnetOffset(-200);

    /////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////

    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits.SupplyCurrentThreshold = 100;
    feederConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1 * 12;
    feederConfig.CurrentLimits.SupplyTimeThreshold = 0.02;
    feederConfig.CurrentLimits.SupplyCurrentLimit = 40;
    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    feederControllerConfig = new Slot0Configs().withKP(0.002).withKV(0.12);
    feederConfig.Slot0 = feederControllerConfig;

    feederMotor.getConfigurator().apply(feederConfig, 0.05);

    /////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.CurrentLimits.SupplyCurrentThreshold = 100;
    shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1 * 12;
    shooterConfig.CurrentLimits.SupplyTimeThreshold = 0.02;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = 40;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterControllerConfig = new Slot0Configs().withKP(2).withKV(.160);
    shooterConfig.Slot0 = shooterControllerConfig;

    upMotor.getConfigurator().apply(shooterConfig, 0.05);
    downMotor.getConfigurator().apply(shooterConfig, 0.05);

    /////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////

    stickMotor.configFactoryDefault();
    stickMotor.config_kP(0, 0.225);
    stickMotor.setSelectedSensorPosition(0);
    stickMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
    stickMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
    stickMotor.configMotionCruiseVelocity(10000);
    stickMotor.configMotionAcceleration(10000);
    stickMotor.setNeutralMode(NeutralMode.Brake);

    /////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////

    hoodAngleMotor.configFactoryDefault();
    hoodAngleMotor.setSelectedSensorPosition(0);
    hoodAngleMotor.setNeutralMode(NeutralMode.Brake);
    hoodAngleMotor.setInverted(true);

    hoodAngleEncoder.configAllSettings(_canCoderConfigurationAngle);
    hoodAngleEncoder.configMagnetOffset(-200);

    /////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////

    climbLMotor.configFactoryDefault();
    climbLMotor.config_kP(0, 0.01);
    climbLMotor.setSelectedSensorPosition(0);
    climbLMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
    climbLMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
    climbLMotor.configMotionCruiseVelocity(100000);
    climbLMotor.configMotionAcceleration(100000);
    climbLMotor.setNeutralMode(NeutralMode.Brake);
    climbLMotor.setInverted(false);

    climbRMotor.configFactoryDefault();
    climbRMotor.config_kP(0, 0.01);
    climbRMotor.setSelectedSensorPosition(0);
    climbRMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
    climbRMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
    climbRMotor.configMotionCruiseVelocity(100000);
    climbRMotor.configMotionAcceleration(100000);
    climbRMotor.setNeutralMode(NeutralMode.Brake);
    climbRMotor.setInverted(true);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setIntakePosition(double position) {
    m_IntakePidController.setReference(MathUtil.clamp(position, 6, 48), ControlType.kSmartMotion);
  }

  public double getIntakeAngle() {
    return intakeAngleEncoder.getAbsolutePosition();
  }

  public void setFeederVelocity(double RPS) {
    feederMotor.setControl(m_feederVoltageVelocity.withVelocity(RPS).withEnableFOC(true));
  }

  public boolean getSensorFeeder() {
    return sensorInput.get();
  }

  public boolean getSensorIntake() {
    return sensorIntake.get();
  }

  public double getSensorShooter() {
    return shooterSensor.getIR();
  }

  public void setShooterVelocity(double RPS) {
    upMotor.setControl(m_shooterVelocityTorque.withVelocity(RPS));
    downMotor.setControl(m_shooterVelocityTorque.withVelocity(RPS));
  }

  public void setShooterVelocityAmf(double RPS, double RPS2) {
    upMotor.setControl(m_shooterVelocityTorque.withVelocity(RPS));
    downMotor.setControl(m_shooterVelocityTorque.withVelocity(RPS2));
  }

  public double getShooterRPS() {
    return ((upMotor.getVelocity().getValue() + downMotor.getVelocity().getValue()) / 2);
  }

  public double getCalculatedShooterRPS(double range) {
    double valueToReturn = MathUtil.clamp(Constants.Shoot.kRPSMap.getInterpolated(new InterpolatingDouble(range)).value,
        44, 90);
    return valueToReturn;
  }

  public void setStickPosition(double position) {
    stickMotor.set(ControlMode.MotionMagic,
        ((MathUtil.clamp(position, Constants.Stick.stowPos, Constants.Stick.openPos) / 360) * 6) * 2048);
  }

  public double getStickPosition() {
    return ((stickMotor.getSelectedSensorPosition() / 2048) / 6) * 360;
  }

  public void setHoodAngle(double angle) {
    double hoodMotorVal = hoodPID.calculate(hoodAngleEncoder.getAbsolutePosition(), angle);
    hoodMotorVal = MathUtil.clamp(hoodMotorVal, Constants.Angle.hoodPIDMin, Constants.Angle.hoodPIDMax);
    if (angle >= Constants.Angle.hoodMinAngle && angle <= Constants.Angle.hoodMaxAngle) {
      hoodAngleMotor.set(hoodMotorVal);
    }
  }

  public double getHoodAngle() {
    return hoodAngleEncoder.getAbsolutePosition();
  }

  public double getCalculatedHoodAngle(double range) {
    double valueToReturn = MathUtil
        .clamp(Constants.Shoot.kHoodMap.getInterpolated(new InterpolatingDouble(range)).value, 149.5, 175.25);
    return valueToReturn;
  }

  public void setSystem(boolean intakeStatus, boolean isHoodReady) {
    if (intakeStatus) {
      intakeLock = true;
    }
    double feederSpeed;
    if (intakeLock) {
      blink(255, 0, 255, 10);
      feederSpeed = 8;
      if (getSensorIntake()) {
        intakeStatus = false;
        setIntakePosition(7);
      } else {
        setIntakePosition(47.2);
      }
    } else {
      if (RobotState.isEnabled() && !intakeLock && !Robot.sourceLock && !Robot.amfLock) {
        setLarson(255, 255, 0, 3, 0.5, 1);
      }
      feederSpeed = 0;
      intakeStatus = false;
      setIntakePosition(7);
    }
    if (!isHoodReady) {
      if (!getSensorFeeder()) {
        setFeederVelocity(feederSpeed);
        if (getSensorIntake()) {
          setFeederVelocity(7);
        }
      } else {
        intakeLock = false;
        if (getSensorShooter() >= Constants.Shooter.sensorDistance) {
          setFeederVelocity(0);
        } else {
          setFeederVelocity(7);
        }
      }
      if (intakeLock) {
        double intakeSpeed = 1;
        setIntakeSpeed(intakeSpeed);
      } else {
        setIntakeSpeed(0);
      }
    } else {
      setFeederVelocity(61);
    }
  }

  public boolean checkShootingStatus(boolean shoot) {
    boolean valueToReturn = false;
    if (shoot) {
      setShooterVelocity(getCalculatedShooterRPS(LimelightHelpers.getTY(Constants.Limelight.kLowerName)));
      setHoodAngle(getCalculatedHoodAngle(LimelightHelpers.getTY(Constants.Limelight.kLowerName)));

      if (LimelightHelpers.getTV(Constants.Limelight.kLowerName)) {
        TargetSeenInTheFrame = true;
      }

      if (TargetSeenInTheFrame) {
        rotPIDValue = -chassisPID(true, 0);
      } else {
        rotPIDValue = 0;
      }
      IsShooterReady = Conversions.epsilonEquals(getShooterRPS(),
          getCalculatedShooterRPS(LimelightHelpers.getTY(Constants.Limelight.kLowerName)), 5);
      IsThereAnyTarget = LimelightHelpers.getTV(Constants.Limelight.kLowerName);
      IsRobotCentered = Conversions.epsilonEquals(LimelightHelpers.getTX(Constants.Limelight.kLowerName), 0, 2);
      IsHoodReady = Conversions.epsilonEquals(hoodAngleEncoder.getAbsolutePosition(),
          getCalculatedHoodAngle(LimelightHelpers.getTY(Constants.Limelight.kLowerName)), 1);
      ready = IsHoodReady && IsRobotCentered && IsShooterReady && IsThereAnyTarget;
      if (ready) {
        valueToReturn = true;
        prepareLock = false;
        setColor(0, 255, 0);
      } else {
        valueToReturn = false;
        blink(0, 0, 255, 10);
      }
    } else {
      TargetSeenInTheFrame = false;
      IsShooterReady = false;
      IsThereAnyTarget = false;
      IsRobotCentered = false;
      IsHoodReady = false;
      ready = false;
      valueToReturn = false;
      rotPIDValue = 0;
      SnapPIDValue = 0;
      if (getSensorFeeder() && LimelightHelpers.getTV(Constants.Limelight.kLowerName)) {
        setShooterVelocity(20);
      } else {
        setShooterVelocity(10);
      }
      setHoodAngle(150.00);
      RobotContainer.m_Superstructure.chassisPID(false, rotPIDSetpoint);
    }
    return valueToReturn;
  }

  public void setClimbPosition(double position) {
    climbLMotor.set(ControlMode.MotionMagic, MathUtil.clamp(position, -1000, 63000));
    climbRMotor.set(ControlMode.MotionMagic, MathUtil.clamp(position, -1000, 63000));
  }

  public void setClimbSpeed(double speed) {
    climbLMotor.set(speed);
    climbRMotor.set(speed);
  }

  public void setClimbStop() {
    climbLMotor.stopMotor();
    climbRMotor.stopMotor();
  }

  public double chassisPID(boolean enabled, double angle) {
    double valueToReturn;
    if (enabled) {
      valueToReturn = MathUtil.clamp(
          rotationPID.calculate(-LimelightHelpers.getTX(Constants.Limelight.kLowerName), angle),
          Constants.Swerve.AlingPids.rotPIDMin, Constants.Swerve.AlingPids.rotPIDMax); // *kminimizer
    } else {
      valueToReturn = 0;
    }
    return valueToReturn;
  }

  public double SnapPID(boolean enabled, double angle) {
    double valueToReturn;
    if (enabled) {
      valueToReturn = MathUtil.clamp(SnapPID.calculate(RobotContainer.m_Swerve.getHeading().getDegrees(), angle),
          Constants.Swerve.AlingPids.SnapPIDMin, Constants.Swerve.AlingPids.SnapPIDMax);
    } else {
      valueToReturn = 0;
    }
    return valueToReturn;
  }

  public double AlingXPID(boolean enabled, double pos) {
    double valueToReturn;
    if (enabled) {
      valueToReturn = MathUtil.clamp(AlingXPID.calculate(LimelightHelpers.getTX(Constants.Limelight.kUpperName), 0),
          Constants.Swerve.AlingPids.AlingXPIDMin, Constants.Swerve.AlingPids.AlingXPIDMax);
    } else {
      valueToReturn = 0;
    }
    return valueToReturn;
  }

  public double AlignYPID(boolean enabled, double area) {
    double valueToReturn;
    if (enabled) {
      valueToReturn = MathUtil.clamp(AlignYPID.calculate(LimelightHelpers.getTY(Constants.Limelight.kUpperName), area),
          Constants.Swerve.AlingPids.AlignYPIDMin, Constants.Swerve.AlingPids.AlignYPIDMax);
    } else {
      valueToReturn = 0;
    }
    return valueToReturn;
  }

  public void zeroGyro() {
    RobotContainer.m_Swerve.gyro.setYaw(0);
  }

  public void autoRot() {
    double translationSup = 0;
    double strafeSup = 0;
    double rotationSup = 0;

    double translationVal = MathUtil.applyDeadband(translationSup * 1, Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup * 1, Constants.stickDeadband);
    double rotationVal = -MathUtil.applyDeadband(rotationSup * 1, Constants.stickDeadband) + rotPIDValue + SnapPIDValue;

    RobotContainer.m_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity, true, true);
  }

  public void setColor(int red, int green, int blue) {
    if (RobotState.isEnabled()) {
      mLed.animate(null);
      mLed.setLEDs(red, green, blue);
    }
  }

  public void setLarson(int r, int g, int b, int size, double speed, int brightness) {
    mLed.animate(new LarsonAnimation(r, g, b, brightness, speed, Constants.Led.LedCount, BounceMode.Front, size));
  }

  public void setRainbow(int brightness, double speed) {
    mLed.animate(new RainbowAnimation(brightness, speed, Constants.Led.LedCount));
  }

  public void blink(int r, int g, int b, int blinkFrequency) {
    if (RobotState.isEnabled()) {
      if (blinkFrequency != 0) {
        if (o % blinkFrequency == 0) {
          setColor(r, g, b);
          o = 1;
        } else if (o % (blinkFrequency / 2) == 0) {
          setColor(0, 0, 0);
        }
        o++;
      } else {
        setColor(r, g, b);
      }
    }
  }

  public void setAni(Animation ani) {
    mLed.animate(ani);
  }

  public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  @Override
  public void periodic() {
    mappedAbsVal = map(getIntakeAngle(), Constants.Intake.absoluteMinIntake, Constants.Intake.absoluteMaxIntake,
        Constants.Intake.incrementalMinIntake, Constants.Intake.incrementalMaxIntake);
    m_IntakeEncoder.setPosition(mappedAbsVal);
    SmartDashboard.putBoolean("Hood Hazırmı", IsHoodReady);
    SmartDashboard.putBoolean("Robot Ortalandımı", IsRobotCentered);
    SmartDashboard.putBoolean("Shooter Hazırmı", IsShooterReady);
    SmartDashboard.putBoolean("Hedef Varmı", IsThereAnyTarget);
    SmartDashboard.putBoolean("Robot Fırlatmaya Hazır", ready);
    SmartDashboard.putNumber("Tırmanma L Pozisyon", climbLMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Tırmanma R Pozisyon", climbRMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Intake AÇI ENCODER", m_IntakeEncoder.getPosition());
    SmartDashboard.putNumber("Çubuk Pos", getStickPosition());
    SmartDashboard.putNumber("Target Area", LimelightHelpers.getTA(Constants.Limelight.kUpperName));
  }
}