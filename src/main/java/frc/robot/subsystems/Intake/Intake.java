package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
    deployCfg.smartCurrentLimit(40);

    deployMotor.configure(
        deployCfg,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    SparkFlexConfig rollerCfg = new SparkFlexConfig();
    rollerCfg.idleMode(IdleMode.kCoast);
    rollerCfg.inverted(false);
    rollerCfg.smartCurrentLimit(60);

    roller1Motor.configure(
        rollerCfg,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    roller2Motor.configure(
        rollerCfg,
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











/* 
package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("removal")
public class Intake extends SubsystemBase {
  private static final int kDeployCanId = 20;
  private static final int kRoller1CanId = 21;
  private static final int kRoller2CanId = 22;

  private final SparkMax deployMotor = new SparkMax(kDeployCanId, MotorType.kBrushless);
  private final SparkMax roller1Motor = new SparkMax(kRoller1CanId, MotorType.kBrushless);
  private final SparkMax roller2Motor = new SparkMax(kRoller2CanId, MotorType.kBrushless);

  private static final double kDeadband = 0.02;

  private static final double kDeployMaxVolts = 8.0;
  private static final double kDeployMinVoltsToMove = 1.5;

  private static final double kRollerMaxVolts = 12.0;
  private static final double kRollerMinVoltsToMove = 2.0;

  public Intake() {
    SparkMaxConfig deployCfg = new SparkMaxConfig();
    deployCfg.idleMode(IdleMode.kCoast);
    deployCfg.inverted(false);
    deployCfg.smartCurrentLimit(30);

    deployMotor.configure(
        deployCfg,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    SparkMaxConfig rollerCfg = new SparkMaxConfig();
    rollerCfg.idleMode(IdleMode.kCoast);
    rollerCfg.inverted(false);
    rollerCfg.smartCurrentLimit(40);

    roller1Motor.configure(
        rollerCfg,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    roller2Motor.configure(
        rollerCfg,
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

  public Command deployOut(double volts) {
    return runEnd(() -> setDeployVolts(volts), this::stopDeploy);
  }

  public Command deployIn(double volts) {
    return runEnd(() -> setDeployVolts(-Math.abs(volts)), this::stopDeploy);
  }

  public Command rollersIn(double volts) {
    return runEnd(() -> setRollerVolts(Math.abs(volts)), this::stopRollers);
  }

  public Command rollersOut(double volts) {
    return runEnd(() -> setRollerVolts(-Math.abs(volts)), this::stopRollers);
  }

  public Command intake(double deployVolts, double rollerVolts) {
    return runEnd(
        () -> {
          setDeployVolts(deployVolts);
          setRollerVolts(rollerVolts);
        },
        this::stopAll
    );
  }

  public Command outtake(double deployVolts, double rollerVolts) {
    return runEnd(
        () -> {
          setDeployVolts(-Math.abs(deployVolts));
          setRollerVolts(-Math.abs(rollerVolts));
        },
        this::stopAll
    );
  }
}
*/