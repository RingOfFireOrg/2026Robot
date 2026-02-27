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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");

  private final GenericEntry sbDeployOutVolts = tab.add("Deploy Out Volts (manual)", 3.5).getEntry();
  private final GenericEntry sbDeployInVolts = tab.add("Deploy In Volts (manual)", 6.0).getEntry();

//private final GenericEntry sbRollersInVolts = tab.add("Rollers In Volts", 3.0).getEntry();
//private final GenericEntry sbRollersOutVolts = tab.add("Rollers Out Volts", 3.0).getEntry();

  private final GenericEntry sbDeployOutDeg = tab.add("Deploy Out (deg)", 75.0).getEntry();
  private final GenericEntry sbDeployInDeg = tab.add("Deploy In (deg)", 3.0).getEntry();

  private final GenericEntry sbDeployPosDeg = tab.add("Deploy Pos (deg)", 0.0).getEntry();
  private final GenericEntry sbDeployPosMotorRot = tab.add("Deploy Pos (motor rot)", 0.0).getEntry();

  private final GenericEntry sbDeploySpeedDegPerSec = tab.add("Sped", 300.0).getEntry();

  private final GenericEntry sbRollersInPercent = tab.add("Rollers In %", 0.35).getEntry();
  private final GenericEntry sbRollersOutPercent = tab.add("Rollers Out %", 0.35).getEntry();

  private double goalMotorRot = 0.0;
  private boolean goalActive = false;
  private long lastUs = 0;

  public Intake() {
    SparkFlexConfig deployCfg = new SparkFlexConfig();
    deployCfg.idleMode(IdleMode.kCoast);
    deployCfg.inverted(false);
    deployCfg.smartCurrentLimit(60);

    deployCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kDeployP, kDeployI, kDeployD, kDeployFF);

    deployMotor.configure(
        deployCfg,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    deployEncoder.setPosition(0.0);

    tab.add("Zero Deploy Encoder", new InstantCommand(this::zeroDeployEncoder, this));

    SparkFlexConfig rollerCfg1 = new SparkFlexConfig();
    rollerCfg1.idleMode(IdleMode.kCoast);
    rollerCfg1.inverted(false);
    rollerCfg1.smartCurrentLimit(60);
    roller1Motor.configure(
        rollerCfg1,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    SparkFlexConfig rollerCfg2 = new SparkFlexConfig();
    rollerCfg2.idleMode(IdleMode.kCoast);
    rollerCfg2.inverted(true);
    rollerCfg2.smartCurrentLimit(60);
    roller2Motor.configure(
        rollerCfg2,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    stopAll();
  }

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

  //blic void stopRollers() {
 // roller1Motor.setVoltage(0.0);
//  roller2Motor.setVoltage(0.0);
//}

  public void stopRollers() {
    roller1Motor.set(0.0);
    roller2Motor.set(0.0);
  }

  public void stopAll() {
    stopDeploy();
    stopRollers();
  }

  public Command deployOut() {
    return runOnce(() -> setDeployPositionDeg(sbDeployOutDeg.getDouble(70.0)));
  }

  public Command retractIn() {
    return runOnce(() -> setDeployPositionDeg(sbDeployInDeg.getDouble(4.0)));
  }

  public Command deployOutManual() {
    return runEnd(
        () -> setDeployVolts(Math.abs(sbDeployOutVolts.getDouble(3.5))),
        this::stopDeploy
    );
  }

  public Command retractInManual() {
    return runEnd(
        () -> setDeployVolts(-Math.abs(sbDeployInVolts.getDouble(6.0))),
        this::stopDeploy
    );
  }

  public Command rollersIn() {
    return runEnd(
      () -> setRollerPercent(Math.abs(sbRollersInPercent.getDouble(0.35))),
      this::stopRollers
   );
  }

public Command rollersOut() {
  return runEnd(
      () -> setRollerPercent(-Math.abs(sbRollersOutPercent.getDouble(0.35))),
      this::stopRollers
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
    sbDeployPosMotorRot.setDouble(motorRot);
    sbDeployPosDeg.setDouble(motorRotToDeg(motorRot));

    if (!goalActive) return;

    double speedDegPerSec = MathUtil.clamp(sbDeploySpeedDegPerSec.getDouble(45.0), 1.0, 360.0);
    double maxMotorRotPerSec = degPerSecToMotorRotPerSec(speedDegPerSec);
    double maxStep = maxMotorRotPerSec * dt;

    double err = goalMotorRot - motorRot;
    double step = MathUtil.clamp(err, -maxStep, maxStep);
    double next = motorRot + step;

    deployController.setReference(next, ControlType.kPosition);
  }
}



















/*import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("removal")
public class Intake extends SubsystemBase {
  private static final int kDeployCanId = 20;
  private static final int kRoller1CanId = 21;
  private static final int kRoller2CanId = 22;

  private final SparkFlex deployMotor = new SparkFlex(kDeployCanId, MotorType.kBrushless);
  private final SparkFlex roller1Motor = new SparkFlex(kRoller1CanId, MotorType.kBrushless);
  private final SparkFlex roller2Motor = new SparkFlex(kRoller2CanId, MotorType.kBrushless);

  private static final double kDeadband = 0.02;

  private static final double kDeployMaxVolts = 8.0;
  private static final double kDeployMinVoltsToMove = 1.5;

  private static final double kRollerMaxVolts = 12.0;
  private static final double kRollerMinVoltsToMove = 2.0;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");
  private final GenericEntry sbDeployOutVolts = tab.add("Deploy Out Volts", 3.5).getEntry();
  private final GenericEntry sbDeployInVolts = tab.add("Deploy In Volts", 6.0).getEntry();
  private final GenericEntry sbRollersInVolts = tab.add("Rollers In Volts", 8.0).getEntry();
  private final GenericEntry sbRollersOutVolts = tab.add("Rollers Out Volts", 8.0).getEntry();

  public Intake() {
    SparkFlexConfig deployCfg = new SparkFlexConfig();
    deployCfg.idleMode(IdleMode.kCoast);
    deployCfg.inverted(false);
    deployCfg.smartCurrentLimit(60);

    deployMotor.configure(
        deployCfg,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    SparkFlexConfig rollerCfg1 = new SparkFlexConfig();
    rollerCfg1.idleMode(IdleMode.kCoast);
    rollerCfg1.inverted(false);
    rollerCfg1.smartCurrentLimit(60);

    roller1Motor.configure(
        rollerCfg1,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    SparkFlexConfig rollerCfg2 = new SparkFlexConfig();
    rollerCfg2.idleMode(IdleMode.kCoast);
    rollerCfg2.inverted(true);
    rollerCfg2.smartCurrentLimit(60);

    roller2Motor.configure(
        rollerCfg2,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    stopAll();
  }

  private static double clampVolts(double volts, double maxVolts, double minVoltsToMove) {
    double cmd = MathUtil.applyDeadband(volts, kDeadband);
    cmd = MathUtil.clamp(cmd, -maxVolts, maxVolts);
    if (Math.abs(cmd) > 1e-6) {
      cmd = Math.copySign(Math.max(Math.abs(cmd), minVoltsToMove), cmd);
    }
    return cmd;
  }

  public void setDeployVolts(double volts) {
    deployMotor.setVoltage(clampVolts(volts, kDeployMaxVolts, kDeployMinVoltsToMove));
  }

  public void setRollerVolts(double volts) {
    double cmd = clampVolts(volts, kRollerMaxVolts, kRollerMinVoltsToMove);
    roller1Motor.setVoltage(cmd);
    roller2Motor.setVoltage(cmd);
  }

  public void stopDeploy() {
    deployMotor.setVoltage(0.0);
  }

  public void stopRollers() {
    roller1Motor.setVoltage(0.0);
    roller2Motor.setVoltage(0.0);
  }

  public void stopAll() {
    stopDeploy();
    stopRollers();
  }

  public Command deployOut() {
    return runEnd(
        () -> setDeployVolts(Math.abs(sbDeployOutVolts.getDouble(3.5))),
        this::stopDeploy
    );
  }

  public Command retractIn() {
    return runEnd(
        () -> setDeployVolts(-Math.abs(sbDeployInVolts.getDouble(6.0))), 
        this::stopDeploy
    );
  }

  public Command rollersIn() {
    return runEnd(
        () -> setRollerVolts(Math.abs(sbRollersInVolts.getDouble(8.0))),
        this::stopRollers
    );
  }

  public Command rollersOut() {
    return runEnd(
        () -> setRollerVolts(-Math.abs(sbRollersOutVolts.getDouble(8.0))),
        this::stopRollers
    );
  }
}
*/








