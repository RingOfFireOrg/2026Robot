package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;


@SuppressWarnings("removal")
public class Intake extends SubsystemBase {
  private static final int kDeployCanId = 20;
  private static final int kRoller1CanId = 21;
  private static final int kRoller2CanId = 22;

  private final SparkFlex deployMotor = new SparkFlex(kDeployCanId, MotorType.kBrushless);
  private final SparkFlex roller1Motor = new SparkFlex(kRoller1CanId, MotorType.kBrushless);
  private final SparkFlex roller2Motor = new SparkFlex(kRoller2CanId, MotorType.kBrushless);

  private final RelativeEncoder deployEncoder = deployMotor.getEncoder();
  private final SparkClosedLoopController deployController = deployMotor.getClosedLoopController();
  private final SparkFlexConfig deployRuntimeCfg = new SparkFlexConfig();
  private IdleMode currentDeployIdleMode = IdleMode.kBrake;

  private static final double kDeadband = 0.02;
  private static final double kDeployMaxVolts = 8.0;
  private static final double kDeployMinVoltsToMove = 1.5;

  private static final double kRollerMaxVolts = 12.0;
  private static final double kRollerMinVoltsToMove = 2.0;
  private static final double kDeployGearRatio = 45.0;
  private static final double kDeployP = 0.15;
  private static final double kDeployI = 0.0;
  private static final double kDeployD = 0.0;
  private static final double kDeployFF = 0.0;

  private static final double kDeployOutVolts = 3.5;
  private static final double kDeployInVolts = 6.0;
  private static final double kDeployOutDeg = 83.0;
  private static final double kShakeDeg = 50.0;
  private static final double kDeployInDeg = 3.0;
  private static final double kDeploySpeedDegPerSec = 500.0;
  private static final double kRollersInPercent = 0.5;
  //private static final double kRollersOutPercent = 0.5;
  private static final double kShakeLowDeg = 50.0;
  private static final double kShakeHighDeg = 83.0;
  private static final double kShakeWaitSec = 0.1;



  private double goalMotorRot = 0.0;
  private boolean goalActive = false;
  private long lastUs = 0;

  public Intake() {
    SparkFlexConfig deployCfg = new SparkFlexConfig();
    deployCfg.idleMode(IdleMode.kBrake);
    deployCfg.inverted(false);
    deployCfg.smartCurrentLimit(30);

    deployCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kDeployP, kDeployI, kDeployD, kDeployFF);

    deployMotor.configure(
        deployCfg,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    deployEncoder.setPosition(0.0);

    //tab.add("Zero Deploy Encoder", new InstantCommand(this::zeroDeployEncoder, this));

    SparkFlexConfig rollerCfg1 = new SparkFlexConfig();
    rollerCfg1.idleMode(IdleMode.kCoast);
    rollerCfg1.inverted(false);
    rollerCfg1.smartCurrentLimit(40);
    roller1Motor.configure(
        rollerCfg1,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    SparkFlexConfig rollerCfg2 = new SparkFlexConfig();
    rollerCfg2.idleMode(IdleMode.kCoast);
    rollerCfg2.inverted(true);
    rollerCfg2.smartCurrentLimit(40);
    roller2Motor.configure(
        rollerCfg2,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    stopAll();
  }

  @SuppressWarnings("unused")
  private void zeroDeployEncoder() {
    deployEncoder.setPosition(0.0);
    goalMotorRot = 0.0;
    goalActive = false;
    lastUs = 0;
  }

  private static double clampVolts(double volts, double maxVolts, double minVoltsToMove) {
    double cmd = MathUtil.applyDeadband(volts, kDeadband);
    cmd = MathUtil.clamp(cmd, -maxVolts, maxVolts);
    if (Math.abs(cmd) > 1e-6) {
      cmd = Math.copySign(Math.max(Math.abs(cmd), minVoltsToMove), cmd);
    }
    return cmd;
  }

  private double motorRotToDeg(double motorRot) {
    return motorRot * (360.0 / kDeployGearRatio);
  }

  private double degToMotorRot(double deg) {
    return (deg / 360.0) * kDeployGearRatio;
  }

  private double degPerSecToMotorRotPerSec(double degPerSec) {
    return (degPerSec / 360.0) * kDeployGearRatio;
  }

  public void setDeployVolts(double volts) {
    deployMotor.setVoltage(clampVolts(volts, kDeployMaxVolts, kDeployMinVoltsToMove));
  }

  public void setRollerVolts(double volts) {
    double cmd = clampVolts(volts, kRollerMaxVolts, kRollerMinVoltsToMove);
    roller1Motor.setVoltage(cmd);
    roller2Motor.setVoltage(cmd);
  }

  public void setRollerPercent(double percent) {
    double cmd = MathUtil.applyDeadband(percent, kDeadband);
    cmd = MathUtil.clamp(cmd, -1.0, 1.0);

    roller1Motor.set(cmd);
    roller2Motor.set(cmd);
  }
  

  public void setDeployPositionDeg(double targetDeg) {
    goalMotorRot = degToMotorRot(targetDeg);
    goalActive = true;
  }

  public void stopDeploy() {
    goalActive = false;
    deployMotor.setVoltage(0.0);
  }

  public void stopRollers() {
    roller1Motor.set(0.0);
    roller2Motor.set(0.0);
  }

  public void stopAll() {
    stopDeploy();
    stopRollers();
  }
  private void setDeployIdleMode(IdleMode mode) {
    if (currentDeployIdleMode == mode) return;

    deployRuntimeCfg.idleMode(mode);
    REVLibError err =
      deployMotor.configure(
          deployRuntimeCfg,
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);

  if (err == REVLibError.kOk) {
    currentDeployIdleMode = mode;
  }
}

  public Command deployOut() {
    return runOnce(() -> {
    setDeployIdleMode(IdleMode.kCoast);
    setDeployPositionDeg(kDeployOutDeg);
  });
  }

  public Command Shake() {
    return runOnce(() -> {
    setDeployPositionDeg(kShakeDeg);
  });
  }

  public Command retractIn() {
    return runOnce(() -> {
    setDeployIdleMode(IdleMode.kBrake);
    setDeployPositionDeg(kDeployInDeg);
  });
  }

  public Command deployOutManual() {
    return runEnd(
        () -> setDeployVolts(kDeployOutVolts),
        this::stopDeploy
    );
  }

  public Command retractInManual() {
    return runEnd(
        () -> setDeployVolts(-kDeployInVolts),
        this::stopDeploy
    );
  }

  public Command rollersIn() {
    return runEnd(
      () -> setRollerPercent(kRollersInPercent),
      this::stopRollers
   );
  }

public Command rollersOut() {
  return runEnd(
      () -> setRollerPercent(-0.6),
        //-Math.abs(sbRollersOutPercent.getDouble(0.35))),
      this::stopRollers
   );
  }
public double getDeployPositionDeg() {
  return motorRotToDeg(deployEncoder.getPosition());
}

public Command shakeBalls() {
  return Commands.sequence(
      Commands.runOnce(() -> setDeployPositionDeg(kShakeLowDeg), this),
      Commands.waitSeconds(kShakeWaitSec),
      Commands.runOnce(() -> setDeployPositionDeg(kShakeHighDeg), this),
      Commands.waitSeconds(kShakeWaitSec)
  );
}


  @Override
  public void periodic() {
    long nowUs = System.nanoTime() / 1000L;
    if (lastUs == 0) lastUs = nowUs;
    double dt = (nowUs - lastUs) / 1_000_000.0;
    lastUs = nowUs;
    if (dt <= 0.0) dt = 0.02;

    double motorRot = deployEncoder.getPosition();

    if (Constants.tuningMode) {
        Logger.recordOutput("Intake/DeployDeg", motorRotToDeg(motorRot));
        Logger.recordOutput("Intake/DeployMotorRot", motorRot);
    }

    if (!goalActive) return;

    double speedDegPerSec = MathUtil.clamp(kDeploySpeedDegPerSec, 1.0, 720.0);
    double maxMotorRotPerSec = degPerSecToMotorRotPerSec(speedDegPerSec);
    double maxStep = maxMotorRotPerSec * dt;

    double err = goalMotorRot - motorRot;
    double step = MathUtil.clamp(err, -maxStep, maxStep);
    double next = motorRot + step;

    deployController.setReference(next, ControlType.kPosition);
  }
}








