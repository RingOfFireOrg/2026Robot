package frc.robot.subsystems.Turret;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.LimelightHelpers;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.FeedbackSensor;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.RobotBase;





@SuppressWarnings({ "removal", "unused" })
public class Turret extends SubsystemBase {
  

  
  private static final String kCanBus = "FRC-3459-PT-CANivore";
  private static int hello = 1;
  private static final int kMotorCanId = 40; //turret
  private static final int kShooterCanId = 41;//bottom
  private static final int kShooter2CanId = 42;//top
  private static final int kAnglerCanId = 43;
  //private final InterpolatingDoubleTreeMap shooterRpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap topRpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap bottomRpmMap = new InterpolatingDoubleTreeMap();
  private static final double kMinShotDistM = 1.35;
  private static final double kMaxShotDistM = 4.00;
  private static final int kAnglerCurrentLimit = 30;
  private static final boolean kAnglerInverted = false;
  private static final double kAnglerKpVoltsPerRot = 6.0;
  private static final double kAnglerMaxVolts = 4.0;
  private static final double kAnglerTolRot = 0.01;
  private static final double kDistMinM = 0.75;
  private static final double kDistMaxM = 4.00;
  private static final double kAnglerMinRot = 0.00;
  private static final double kAnglerMaxRot = 0.80;
  private double anglerSetpointRot = 0.0;
  private boolean anglerEnabled = false; 
  private static final double kSpinDeltaRpm = 250.0;
  private static final double kBoostStartRpmErr = 200.0;
  private static final double kBoostFullRpmErr = 900.0;
  private static final double kBoostMaxVolts = 1.0;
  private double lastShooterPrintTime = 0.0;
  private static final double kShooterDefaultKp = 0.10;
  private static final double kShooterDefaultKi = 0.0;
  private static final double kShooterDefaultKd = 0.0;
  private static final double kShooterDefaultKv = 0.112;


  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

  private final GenericEntry sbTopRpm = shooterTab.add("Top RPM", -3000.0).getEntry();
  private final GenericEntry sbBottomRpm = shooterTab.add("Bottom RPM", 3000.0).getEntry();

  private final GenericEntry sbShooterKp = shooterTab.add("kP", kShooterDefaultKp).getEntry();
  private final GenericEntry sbShooterKi = shooterTab.add("kI", kShooterDefaultKi).getEntry();
  private final GenericEntry sbShooterKd = shooterTab.add("kD", kShooterDefaultKd).getEntry();
  private final GenericEntry sbShooterKv = shooterTab.add("kV", kShooterDefaultKv).getEntry();

  private final GenericEntry sbBoostStartErr = shooterTab.add("Boost Start Err RPM", kBoostStartRpmErr).getEntry();
  private final GenericEntry sbBoostFullErr = shooterTab.add("Boost Full Err RPM", kBoostFullRpmErr).getEntry();
  private final GenericEntry sbBoostMaxVolts = shooterTab.add("Boost Max Volts", kBoostMaxVolts).getEntry();
  public final GenericEntry sbTURRET_P = shooterTab.add("Turret_P", 0.10).getEntry();
  public final GenericEntry sbTURRET_D = shooterTab.add("Turret_D", 0.0).getEntry();
  public final GenericEntry trenchOffset = shooterTab.add("Trench Offset Left", 5).getEntry();
  public final GenericEntry HubMiddleOffset = shooterTab.add("Hub Middle Tag Offset", 0.5).getEntry();
  public final GenericEntry HubSideOffset = shooterTab.add("Hub Side Tag Offset", 1).getEntry();


  private double appliedShooterKp = Double.NaN;
  private double appliedShooterKi = Double.NaN;
  private double appliedShooterKd = Double.NaN;
  private double appliedShooterKv = Double.NaN;

  



  private final SparkMax rotMotor = new SparkMax(kMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder rotEncoder = rotMotor.getEncoder();
  

  //private final TalonFX shooterMotor = new TalonFX(kShooterCanId, kCanBus);
  private final TalonFX shooterMotor = new TalonFX(kShooterCanId, kCanBus);

  private final VoltageOut shooterVoltsReq = new VoltageOut(0.0);
  private static final double kShooterMaxVolts = 12.0;

  //private final TalonFX shooterMotor2 = new TalonFX(kShooter2CanId, kCanBus);
  private final TalonFX shooterMotor2 = new TalonFX(kShooter2CanId, kCanBus);

  private final VoltageOut shooter2VoltsReq = new VoltageOut(0.0);
  private static final double kShooter2MaxVolts = 12.0;

  private final VelocityVoltage shooterVelReq = new VelocityVoltage(0.0);
  private final VelocityVoltage shooter2VelReq = new VelocityVoltage(0.0);

  private final SparkMax anglerMotor = new SparkMax(kAnglerCanId, MotorType.kBrushless);
  private final RelativeEncoder eRelativeEncoder = anglerMotor.getEncoder();
  

  private static final boolean kEnableSoftLimits = true;
  private static final double kMinTurretDeg = -90.0;
  private static final double kMaxTurretDeg = 90.0;
  private static final double kTurretToleranceDeg = 2.0;

  private static final double kDeadband = 0.02;
  private static final double kMaxDuty = 0.35;
  private static final double kMinDutyToMove = 0.08;

  private double lastPrintTimeSec = 0.0;
  private double lastCmdDuty = 0.0;

  private final SparkClosedLoopController rotController = rotMotor.getClosedLoopController();
  private static final double kTurretGearRatio = 105.0;
  private static final double kTurretPosP = 0.15;
  private static final double kTurretPosI = 0.0;
  private static final double kTurretPosD = 0.0;

  public Turret() {
    //rotation motor config
    SparkMaxConfig rotConfig = new SparkMaxConfig();
      rotConfig.idleMode(IdleMode.kBrake);
      rotConfig.inverted(false);
      rotConfig.smartCurrentLimit(30);

    rotConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kTurretPosP, kTurretPosI, kTurretPosD);

    rotMotor.configure(
      rotConfig,
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters);
    
    rotEncoder.setPosition(0.0);

    // shooter motor configs
    TalonFXConfiguration shooterCfg = new TalonFXConfiguration();
    TalonFXConfiguration shooter2Cfg = new TalonFXConfiguration();

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = kShooterDefaultKp;
    slot0.kI = kShooterDefaultKi;
    slot0.kD = kShooterDefaultKd;
    slot0.kV = kShooterDefaultKv;

    shooterCfg.Slot0 = slot0;
    shooter2Cfg.Slot0 = slot0;

    //Angler motor rotConfig
    SparkMaxConfig anglerCfg = new SparkMaxConfig();
    anglerCfg.idleMode(IdleMode.kBrake);
    anglerCfg.inverted(false);
    anglerCfg.smartCurrentLimit(kAnglerCurrentLimit);

    anglerMotor.configure(
      anglerCfg,
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters);
    
    eRelativeEncoder.setPosition(0.0);
   
    //Shooter motor 1 output configs
    MotorOutputConfigs shooterOut = new MotorOutputConfigs();
    shooterOut.NeutralMode = NeutralModeValue.Coast;
    shooterOut.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterCfg.MotorOutput = shooterOut;

    //Shooter motor 2 output configs
    MotorOutputConfigs shooterOut2 = new MotorOutputConfigs();
    shooterOut2.NeutralMode = NeutralModeValue.Coast;
    shooterOut2.Inverted = InvertedValue.Clockwise_Positive;
    shooter2Cfg.MotorOutput = shooterOut2;

    //limits for both shooter motors
    CurrentLimitsConfigs shooterLimits = new CurrentLimitsConfigs();
    shooterLimits.SupplyCurrentLimitEnable = true;
    shooterLimits.SupplyCurrentLimit = 70.0;
    shooterCfg.CurrentLimits = shooterLimits;
    shooter2Cfg.CurrentLimits = shooterLimits;

    OpenLoopRampsConfigs shooterRamps = new OpenLoopRampsConfigs();
    shooterRamps.VoltageOpenLoopRampPeriod = 0.10;
    shooterCfg.OpenLoopRamps = shooterRamps;
    shooter2Cfg.OpenLoopRamps = shooterRamps;

    shooterMotor.getConfigurator().apply(shooterCfg);
    shooterMotor2.getConfigurator().apply(shooter2Cfg);
    initShooterRpmMap();
    configureShooterShuffleboard();
    applyShooterPid(
        sbShooterKp.getDouble(kShooterDefaultKp),
        sbShooterKi.getDouble(kShooterDefaultKi),
        sbShooterKd.getDouble(kShooterDefaultKd),
        sbShooterKv.getDouble(kShooterDefaultKv));
  }
private void configureShooterShuffleboard() {
  shooterTab.addNumber("Top Measured RPM", this::getShooterTopMeasuredRpm);
  shooterTab.addNumber("Bottom Measured RPM", this::getShooterBottomMeasuredRpm);
  shooterTab.addNumber("Top RPM Error", () -> getDashboardTopRpm() - getShooterTopMeasuredRpm());
  shooterTab.addNumber("Bottom RPM Error", () -> getDashboardBottomRpm() - getShooterBottomMeasuredRpm());
  shooterTab.addNumber("Top Current", () -> shooterMotor2.getStatorCurrent().getValueAsDouble());
  shooterTab.addNumber("Bottom Current", () -> shooterMotor.getStatorCurrent().getValueAsDouble());
  shooterTab.addNumber("Top Temp C", () -> shooterMotor2.getDeviceTemp().getValueAsDouble());
  shooterTab.addNumber("Bottom Temp C", () -> shooterMotor.getDeviceTemp().getValueAsDouble());
}

private void applyShooterPid(double kP, double kI, double kD, double kV) {
  Slot0Configs slot0 = new Slot0Configs();
  slot0.kP = kP;
  slot0.kI = kI;
  slot0.kD = kD;
  slot0.kV = kV;

  shooterMotor.getConfigurator().apply(slot0);
  shooterMotor2.getConfigurator().apply(slot0);

  appliedShooterKp = kP;
  appliedShooterKi = kI;
  appliedShooterKd = kD;
  appliedShooterKv = kV;
}

private void updateShooterPidFromDashboard() {
  double kP = sbShooterKp.getDouble(kShooterDefaultKp);
  double kI = sbShooterKi.getDouble(kShooterDefaultKi);
  double kD = sbShooterKd.getDouble(kShooterDefaultKd);
  double kV = sbShooterKv.getDouble(kShooterDefaultKv);

  if (kP != appliedShooterKp || kI != appliedShooterKi || kD != appliedShooterKd || kV != appliedShooterKv) {
    applyShooterPid(kP, kI, kD, kV);
  }
}

public double getDashboardTopRpm() {
  return sbTopRpm.getDouble(-3000.0);
}

public double getDashboardBottomRpm() {
  return sbBottomRpm.getDouble(3000.0);
}

  public double getTurretRotations() {
    return rotEncoder.getPosition();
  }

  public double getTurretRPM() {
    return rotEncoder.getVelocity();
  }
  public double turretDegreesToMotorRotations(double turretDeg) {
    return (turretDeg / 360.0) * kTurretGearRatio;
  }

  public double motorRotationsToTurretDegrees(double motorRot) {
    return (motorRot / kTurretGearRatio) * 360.0;
  }

  public double getTurretAngleDeg() {
    return motorRotationsToTurretDegrees(rotEncoder.getPosition());
  }

  public void setTurretAngleDeg(double turretDeg) {
    double clampedDeg = MathUtil.clamp(turretDeg, kMinTurretDeg, kMaxTurretDeg);
    if (RobotBase.isReal()) {
      rotController.setReference(
          turretDegreesToMotorRotations(clampedDeg),
          ControlType.kPosition
      );
    }
  }

  public boolean isTurretAtAngle(double targetDeg, double toleranceDeg) {
    return Math.abs(getTurretAngleDeg() - targetDeg) <= toleranceDeg;
  }

  public Command goToTurretAngle(double turretDeg) {
    double clampedDeg = MathUtil.clamp(turretDeg, kMinTurretDeg, kMaxTurretDeg);
    return run(() -> setTurretAngleDeg(clampedDeg))
      .until(() -> isTurretAtAngle(clampedDeg, kTurretToleranceDeg));
  }
  
  public void setTurret(double duty) {
    rotMotor.set(duty);
  }
  
  public void setDutyCycle(double duty) {
    final double angleDeg = getTurretAngleDeg();

    double cmd = MathUtil.applyDeadband(duty, kDeadband);
    cmd = MathUtil.clamp(cmd, -kMaxDuty, kMaxDuty);

    if (Math.abs(cmd) > 1e-6) {
      cmd = Math.copySign(Math.max(Math.abs(cmd), kMinDutyToMove), cmd);
    }

    if (kEnableSoftLimits) {
      boolean hitMin = angleDeg <= kMinTurretDeg && cmd < 0;
      boolean hitMax = angleDeg >= kMaxTurretDeg && cmd > 0;
      if (hitMin || hitMax) {
        rotMotor.set(0.0);
        rateLimitedPrint(
            "[Turret] SOFT LIMIT hit (angleDeg=" + fmt(angleDeg) + ") cmd=" + fmt(cmd) +
                " min=" + fmt(kMinTurretDeg) + " max=" + fmt(kMaxTurretDeg));
        lastCmdDuty = 0.0;
        return;
      }
    }

    rotMotor.set(cmd);
    lastCmdDuty = cmd;

    rateLimitedPrint(
        "[Turret] cmd=" + fmt(cmd) +
            " in=" + fmt(duty) +
            " angleDeg=" + fmt(angleDeg) +
            " rpm=" + fmt(getTurretRPM()));
  }

  public void zeroTurret() {
    rotEncoder.setPosition(0.0);
  }

  public void stopTurret() {
    rotMotor.set(0.0);
    lastCmdDuty = 0.0;
    rateLimitedPrint("[Turret] stopTurret");
  }

  private void rateLimitedPrint(String msg) {
    double now = Timer.getFPGATimestamp();
    if (now - lastPrintTimeSec > 0.25) {
      System.out.println(msg);
      lastPrintTimeSec = now;
    }
  }

  private static String fmt(double v) {
    return String.format("%.3f", v);
  }

  private static double clamp(double x, double lo, double hi) {
  return Math.max(lo, Math.min(hi, x));
  }

  private static double lerp(double a, double b, double t) {
  return a + (b - a) * t;
  }

public void setShooterVolts(double volts) {
  double cmd = MathUtil.clamp(volts, -kShooterMaxVolts, kShooterMaxVolts);
  shooterMotor.setControl(shooterVoltsReq.withOutput(cmd));//kraken
  //shooterNeo.setVoltage(cmd);                               //neo
  shooterMotor2.setControl(shooter2VoltsReq.withOutput(cmd));

}

public void stopShooter() {
  shooterMotor.setControl(shooterVoltsReq.withOutput(0.0));//kraken
  //shooterNeo.setVoltage(0.0);
  shooterMotor2.setControl(shooter2VoltsReq.withOutput(0.0));

}

public Command runShooterPercent(double percent) {
  return runEnd(() -> setShooterVolts(percent * 12.0), this::stopShooter);
}
/* 
public void setShooterRPM(double topRPM, double bottomRPM) {
  double topRps = topRPM / 60.0;
  double bottomRps = bottomRPM / 60.0;

  shooterMotor.setControl(shooterVelReq.withVelocity(bottomRps));
  shooterMotor2.setControl(shooter2VelReq.withVelocity(topRps));
}
*/
/*public void setShooterRPM(double topRPM, double bottomRPM) {
  double topMeas = getShooterTopMeasuredRpm();
  double botMeas = getShooterBottomMeasuredRpm();

  double topErr = topRPM - topMeas;
  double botErr = bottomRPM - botMeas;

  double boostStartErr = sbBoostStartErr.getDouble(kBoostStartRpmErr);
  double boostFullErr = sbBoostFullErr.getDouble(kBoostFullRpmErr);
  double boostMaxVolts = sbBoostMaxVolts.getDouble(kBoostMaxVolts);

  double topBoost = 0.0;
  if (topErr > boostStartErr) {
    double t = clamp((topErr - boostStartErr) / (boostFullErr - boostStartErr), 0.0, 1.0);
    topBoost = lerp(0.0, boostMaxVolts, t);
  }

  double botBoost = 0.0;
  if (botErr > boostStartErr) {
    double t = clamp((botErr - boostStartErr) / (boostFullErr - boostStartErr), 0.0, 1.0);
    botBoost = lerp(0.0, boostMaxVolts, t);
  }

  double topRps = topRPM / 60.0;
  double botRps = bottomRPM / 60.0;

  shooterMotor2.setControl(shooter2VelReq.withVelocity(topRps).withFeedForward(topBoost));
  shooterMotor.setControl(shooterVelReq.withVelocity(botRps).withFeedForward(botBoost));

  double now = Timer.getFPGATimestamp();
  if (now - lastShooterPrintTime > 0.15) {
    System.out.println(
      "[SHOOT] " +
      "Topset=" + String.format("%.0f", topRPM) +
      " Topmeas=" + String.format("%.0f", topMeas) +
      " Botset=" + String.format("%.0f", bottomRPM) +
      " Botmeas=" + String.format("%.0f", botMeas) +
      " Topboost=" + String.format("%.2f", topBoost) +
      " Botboost=" + String.format("%.2f", botBoost) +
      " Vbat=" + String.format("%.2f", edu.wpi.first.wpilibj.RobotController.getBatteryVoltage())
    );
    lastShooterPrintTime = now;
  }
}*/
public void setShooterRPM(double topRPM, double bottomRPM) {
  double topRps = topRPM / 60.0;
  double botRps = bottomRPM / 60.0;

  shooterMotor2.setControl(shooter2VelReq.withVelocity(topRps));
  shooterMotor.setControl(shooterVelReq.withVelocity(botRps));

  double topMeas = getShooterTopMeasuredRpm();
  double botMeas = getShooterBottomMeasuredRpm();

  double now = Timer.getFPGATimestamp();
  if (now - lastShooterPrintTime > 0.15) {
    System.out.println(
      "[SHOOT] " +
      "Topset=" + String.format("%.0f", topRPM) +
      " Topmeas=" + String.format("%.0f", topMeas) +
      " Botset=" + String.format("%.0f", bottomRPM) +
      " Botmeas=" + String.format("%.0f", botMeas) +
      " Vbat=" + String.format("%.2f", edu.wpi.first.wpilibj.RobotController.getBatteryVoltage())
    );
    lastShooterPrintTime = now;
  }
}

public Command runShooterRPM(DoubleSupplier topRPM, DoubleSupplier bottomRPM) {
  return runEnd(
    () -> setShooterRPM(topRPM.getAsDouble(), bottomRPM.getAsDouble()),
    this::stopShooter
  );
}
private void initShooterRpmMap() {
  topRpmMap.put(1.35, 5450.0);
  topRpmMap.put(1.50, 5250.0);
  topRpmMap.put(2.00, 5100.0);
  topRpmMap.put(2.25, 5150.0);
  topRpmMap.put(3.00, 5450.0);
  topRpmMap.put(3.50, 5700.0);
  topRpmMap.put(4.00, 5950.0);

  bottomRpmMap.put(1.35, 4950.0);
  bottomRpmMap.put(1.50, 4750.0);
  bottomRpmMap.put(2.00, 4600.0);
  bottomRpmMap.put(2.25, 4650.0);
  bottomRpmMap.put(3.00, 4950.0);
  bottomRpmMap.put(3.50, 5200.0);
  bottomRpmMap.put(4.00, 5450.0);
}


public double getTopRpmForDistanceMeters(double distanceM) {
  double d = MathUtil.clamp(distanceM, kMinShotDistM, kMaxShotDistM);
  Double rpm = topRpmMap.get(d);
  return (rpm != null) ? rpm : 5200.0;
}

public double getBottomRpmForDistanceMeters(double distanceM) {
  double d = MathUtil.clamp(distanceM, kMinShotDistM, kMaxShotDistM);
  Double rpm = bottomRpmMap.get(d);
  return (rpm != null) ? rpm : 5200.0;
}


public void setShooterFromDistanceMeters(double distanceM) {
  double topRpm = getTopRpmForDistanceMeters(distanceM);
  double bottomRpm = getBottomRpmForDistanceMeters(distanceM);
  setShooterRPM(topRpm, bottomRpm);
}

public double getAnglerRotations() {
  return eRelativeEncoder.getPosition();
}

public void enableAngler(boolean enabled) {
  anglerEnabled = enabled;
  if (!enabled) {
    anglerMotor.setVoltage(0.0);
  }
}

public void setAnglerDistanceMeters(double distanceM) {
  double d = MathUtil.clamp(distanceM, kDistMinM, kDistMaxM);
  double t = (d - kDistMinM) / (kDistMaxM - kDistMinM);
  anglerSetpointRot = kAnglerMinRot + t * (kAnglerMaxRot - kAnglerMinRot);
}

public void updateAngler() {
  if (!anglerEnabled) {
    return;
  }

  double err = anglerSetpointRot - getAnglerRotations();
  if (Math.abs(err) <= kAnglerTolRot) {
    anglerMotor.setVoltage(0.0);
    return;
  }

  double volts = MathUtil.clamp(err * kAnglerKpVoltsPerRot, -kAnglerMaxVolts, kAnglerMaxVolts);
  anglerMotor.setVoltage(volts);
}

public double getShooterTopMeasuredRpm() {
  //top
  return shooterMotor2.getVelocity().getValueAsDouble() * 60.0;
}

public double getShooterBottomMeasuredRpm() {
  //bottom
  return shooterMotor.getVelocity().getValueAsDouble() * 60.0;
}
public boolean isShooterAtSpeed(double topTolRpm, double bottomTolRpm) {
  return Math.abs(getDashboardTopRpm() - getShooterTopMeasuredRpm()) <= topTolRpm
      && Math.abs(getDashboardBottomRpm() - getShooterBottomMeasuredRpm()) <= bottomTolRpm;
}

public Command runShooterUntilReady(double topTolRpm, double bottomTolRpm) {
  return run(() -> setShooterRPM(getDashboardTopRpm(), getDashboardBottomRpm()))
      .until(() -> isShooterAtSpeed(topTolRpm, bottomTolRpm));
}

public Command holdDashboardShooterRpm() {
  return runEnd(
      () -> setShooterRPM(getDashboardTopRpm(), getDashboardBottomRpm()),
      this::stopShooter);
}

public double getDistanceToTagMeters(String limelightName) {
  Pose3d cam = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);
  double x = cam.getX();
  double z = cam.getZ();
  return Math.hypot(x, z);
}

@Override
public void periodic(){
  updateShooterPidFromDashboard();
  //updateAngler();
}
}
