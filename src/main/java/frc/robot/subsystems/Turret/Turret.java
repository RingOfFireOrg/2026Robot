package frc.robot.subsystems.Turret;
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




@SuppressWarnings({ "removal", "unused" })
public class Turret extends SubsystemBase {
  private static int hello = 1;
  private static final int kMotorCanId = 40;
  private static final int kShooterCanId = 41;
  //private static final int kShooterNeoCanId = 42;
  private static final int kShooter2CanId = 42;
  private static final int kAnglerCanId = 43;



  private final SparkMax motor = new SparkMax(kMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  private final TalonFX shooterMotor = new TalonFX(kShooterCanId);
  private final VoltageOut shooterVoltsReq = new VoltageOut(0.0);
  private static final double kShooterMaxVolts = 12.0;

  private final TalonFX shooterMotor2 = new TalonFX(kShooter2CanId);
  private final VoltageOut shooter2VoltsReq = new VoltageOut(0.0);
  private static final double kShooter2MaxVolts = 12.0;

  private final VelocityVoltage shooterVelReq = new VelocityVoltage(0.0);
  private final VelocityVoltage shooter2VelReq = new VelocityVoltage(0.0);


  //private final SparkMax shooterNeo = new SparkMax(kShooterNeoCanId, MotorType.kBrushless);
  //private static final int kShooterNeoCurrentLimit = 40;

  private final SparkMax angler = new SparkMax(kAnglerCanId, MotorType.kBrushless);
  private final RelativeEncoder eRelativeEncoder = angler.getEncoder();
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

  private static final double kMinRot = -1.0;
  private static final double kMaxRot =  1.0;
  private static final boolean kEnableSoftLimits = false;

  private static final double kDeadband = 0.02;
  private static final double kMaxDuty = 0.35;
  private static final double kMinDutyToMove = 0.08;

  private double lastPrintTimeSec = 0.0;
  private double lastCmdDuty = 0.0;

  public Turret() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.smartCurrentLimit(30);

    motor.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    encoder.setPosition(0.0);
    System.out.println("[Turret] init: CAN=" + kMotorCanId + " inverted=false brake=true");

    TalonFXConfiguration shooterCfg = new TalonFXConfiguration();
    TalonFXConfiguration shooter2Cfg = new TalonFXConfiguration();

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 0.12;
    slot0.kI = 0.0;
    slot0.kD = 0.0;
    slot0.kV = 0.12;

    shooterCfg.Slot0 = slot0;
    shooter2Cfg.Slot0 = slot0;
    
    //SparkMaxConfig shooterNeoCfg = new SparkMaxConfig();
    //shooterNeoCfg.idleMode(IdleMode.kCoast);
    //shooterNeoCfg.inverted(true);
    //shooterNeoCfg.smartCurrentLimit(kShooterNeoCurrentLimit);

    //shooterNeo.configure(
      //shooterNeoCfg,
      //SparkBase.ResetMode.kResetSafeParameters,
      //SparkBase.PersistMode.kPersistParameters);

    SparkMaxConfig anglerCfg = new SparkMaxConfig();
    anglerCfg.idleMode(IdleMode.kBrake);
    anglerCfg.inverted(false);
    anglerCfg.smartCurrentLimit(kAnglerCurrentLimit);

    angler.configure(
      anglerCfg,
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters);
    
      eRelativeEncoder.setPosition(0.0);
   


    MotorOutputConfigs shooterOut = new MotorOutputConfigs();
    shooterOut.NeutralMode = NeutralModeValue.Coast;
    shooterOut.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterCfg.MotorOutput = shooterOut;

    MotorOutputConfigs shooterOut2 = new MotorOutputConfigs();
    shooterOut2.NeutralMode = NeutralModeValue.Coast;
    shooterOut2.Inverted = InvertedValue.Clockwise_Positive;
    shooter2Cfg.MotorOutput = shooterOut2;

    CurrentLimitsConfigs shooterLimits = new CurrentLimitsConfigs();
    shooterLimits.SupplyCurrentLimitEnable = true;
    shooterLimits.SupplyCurrentLimit = 60.0;
    shooterCfg.CurrentLimits = shooterLimits;

    OpenLoopRampsConfigs shooterRamps = new OpenLoopRampsConfigs();
    shooterRamps.VoltageOpenLoopRampPeriod = 0.10;
    shooterCfg.OpenLoopRamps = shooterRamps;

    shooter2Cfg.CurrentLimits = shooterLimits;
    shooter2Cfg.OpenLoopRamps = shooterRamps;

    shooterMotor.getConfigurator().apply(shooterCfg);
    shooterMotor2.getConfigurator().apply(shooter2Cfg);
  }

  public double getMotorRotations() {
    return encoder.getPosition();
  }

  public double getMotorRpm() {
    return encoder.getVelocity();
  }


  public void setDutyCycle(double duty) {
    final double posRot = getMotorRotations();

    double cmd = MathUtil.applyDeadband(duty, kDeadband);
    cmd = MathUtil.clamp(cmd, -kMaxDuty, kMaxDuty);

    if (Math.abs(cmd) > 1e-6) {
      cmd = Math.copySign(Math.max(Math.abs(cmd), kMinDutyToMove), cmd);
    }

    if (kEnableSoftLimits) {
      boolean hitMin = posRot <= kMinRot && cmd < 0;
      boolean hitMax = posRot >= kMaxRot && cmd > 0;
      if (hitMin || hitMax) {
        motor.set(0.0);
        rateLimitedPrint(
            "[Turret] SOFT LIMIT hit (posRot=" + fmt(posRot) + ") cmd=" + fmt(cmd) +
                " min=" + fmt(kMinRot) + " max=" + fmt(kMaxRot));
        lastCmdDuty = 0.0;
        return;
      }
    }

    motor.set(cmd);
    lastCmdDuty = cmd;

    rateLimitedPrint(
        "[Turret] cmd=" + fmt(cmd) +
            " in=" + fmt(duty) +
            " posRot=" + fmt(posRot) +
            " rpm=" + fmt(getMotorRpm()));
  }

  public void stop() {
    motor.set(0.0);
    lastCmdDuty = 0.0;
    rateLimitedPrint("[Turret] stop");
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

public void setShooterRPM(double topRPM, double bottomRPM) {
  double topRps = topRPM / 60.0;
  double bottomRps = bottomRPM / 60.0;

  shooterMotor.setControl(shooterVelReq.withVelocity(topRps));
  shooterMotor2.setControl(shooter2VelReq.withVelocity(bottomRps));
}

public Command runShooterRPM(DoubleSupplier topRPM, DoubleSupplier bottomRPM) {
  return runEnd(
    () -> setShooterRPM(topRPM.getAsDouble(), bottomRPM.getAsDouble()),
    this::stopShooter
  );
}

public double getAnglerRotations() {
  return eRelativeEncoder.getPosition();
}

public void enableAngler(boolean enabled) {
  anglerEnabled = enabled;
  if (!enabled) {
    angler.setVoltage(0.0);
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
    angler.setVoltage(0.0);
    return;
  }

  double volts = MathUtil.clamp(err * kAnglerKpVoltsPerRot, -kAnglerMaxVolts, kAnglerMaxVolts);
  angler.setVoltage(volts);
}

public double getDistanceToTagMeters(String limelightName) {
  Pose3d cam = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);
  double x = cam.getX();
  double z = cam.getZ();
  return Math.hypot(x, z);
}
@Override
public void periodic(){
  updateAngler();
}
}




/* 
package frc.robot.subsystems.Turret;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;

@SuppressWarnings("removal")



public class Turret extends SubsystemBase {
  private final SparkMax motor = new SparkMax(41, MotorType.kBrushless); // CAN ID 20
  private final RelativeEncoder encoder = motor.getEncoder();

  private static final double kMinRot = -1.0;  
  private static final double kMaxRot =  1.0;  
  private static final boolean kEnableSoftLimits = false; // TEMP: disable until abs encoder


public Turret() {
  SparkMaxConfig config = new SparkMaxConfig();
  config.idleMode(IdleMode.kBrake);
  config.inverted(false);

  motor.configure(
      config,
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters
  );

  encoder.setPosition(0.0);
}


  public double getMotorRotations() {
    return encoder.getPosition();
  }

public void setDutyCycle(double output) {
  double pos = getMotorRotations();

  if (kEnableSoftLimits) {
    if ((pos <= kMinRot && output < 0) || (pos >= kMaxRot && output > 0)) {
      motor.set(0.0);
      return;
    }
  }

  motor.set(MathUtil.clamp(output, -0.05, 0.05));
}


  public void stop() {
    motor.set(0.0);
  }
}
*/