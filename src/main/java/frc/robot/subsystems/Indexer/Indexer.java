package frc.robot.subsystems.Indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.function.DoubleSupplier;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.FeedbackSensor;

public class Indexer extends SubsystemBase {
  private static final int kMotorCanId = 32;//indexer

  private final SparkMax motor = new SparkMax(kMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  private static final double kDeadband = 0.02;
  private static final double kMaxVolts = 8.0;
  private static final double kMinVoltsToMove = 1.5;


  private static final double kDefaultKp = 0.0005;
  private static final double kDefaultKi = 0.0;
  private static final double kDefaultKd = 0.0;
  private static final double kDefaultKff = 0.0002;

  private static final double kFeedPercent = 0.90; 
  private static final double kReverseFeedPercent = -0.90; 
  private static final double kManualVolts = 4.0; 
  private static final double kFeedRPM = 3000.0;
  private static final double kReverseFeedRPM = -2000.0;

  private double appliedKp = Double.NaN;
  private double appliedKi = Double.NaN;
  private double appliedKd = Double.NaN;
  private double appliedKff = Double.NaN;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Indexer");

  private GenericEntry sbFeedPercent;
  private GenericEntry sbReversePercent;
  private GenericEntry sbManualVolts;

  private GenericEntry sbFeedRpm;
  private GenericEntry sbReverseRpm;

  private GenericEntry sbKp;
  private GenericEntry sbKi;
  private GenericEntry sbKd;
  private GenericEntry sbKff;

  @SuppressWarnings("removal")
  public Indexer() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.inverted(false);
    config.smartCurrentLimit(30);
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(kDefaultKp, kDefaultKi, kDefaultKd, kDefaultKff);

    tuningInitialization();

    motor.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    encoder.setPosition(0.0);
    appliedKp = kDefaultKp;
    appliedKi = kDefaultKi;
    appliedKd = kDefaultKd;
    appliedKff = kDefaultKff;
  }

  // tuningIntialization adds ShuffleBoard widgets used for robot tuning 
  private void tuningInitialization() {

    // Shuffleboard widget updates impact system performance as more variables are added.
    // Constants.tuningMode needs to be false unless you are tuning.
    if (Constants.tuningMode) {
      sbFeedPercent = tab.add("Feed %", kFeedPercent).getEntry();
      sbReversePercent = tab.add("Reverse %", -kReverseFeedPercent).getEntry();
      sbManualVolts = tab.add("Manual Volts", kManualVolts).getEntry();
      sbFeedRpm = tab.add("Feed RPM", kFeedRPM).getEntry();
      sbReverseRpm = tab.add("Reverse RPM", kReverseFeedRPM).getEntry();

      sbKp = tab.add("Indexer kP", kDefaultKp).getEntry();
      sbKi = tab.add("Indexer kI", kDefaultKi).getEntry();
      sbKd = tab.add("Indexer kD", kDefaultKd).getEntry();
      sbKff = tab.add("Indexer kFF", kDefaultKff).getEntry();

      tab.addNumber("RPM", this::getMotorRpm);
      tab.addNumber("Rotations", this::getMotorRotations);
      tab.addNumber("Applied Volts", this::getAppliedVolts);
      tab.addNumber("Current", motor::getOutputCurrent);
      tab.addNumber("Motor Temp C", motor::getMotorTemperature);
      tab.addNumber("RPM Error", () -> getFeedRpm() - getMotorRpm());
    }

  }

@Override
public void periodic() {
  updatePidFromDashboard();
  if (getMotorRpm() < 50 && motor.getOutputCurrent() > 25) {
    runPercent(-0.3);
  }
}

  public double getMotorRotations() {
    return encoder.getPosition();
  }

  public double getMotorRpm() {
    return encoder.getVelocity();
  }

  public void setVolts(double volts) {
    double cmd = MathUtil.applyDeadband(volts, kDeadband);
    cmd = MathUtil.clamp(cmd, -kMaxVolts, kMaxVolts);

    if (Math.abs(cmd) > 1e-6) {
      cmd = Math.copySign(Math.max(Math.abs(cmd), kMinVoltsToMove), cmd);
    }

    motor.setVoltage(cmd);
  }

  @SuppressWarnings("removal")
  private void applyPid(double kP, double kI, double kD, double kFF) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kP, kI, kD, kFF);

    motor.configure(
      config,
      SparkBase.ResetMode.kNoResetSafeParameters,
      SparkBase.PersistMode.kNoPersistParameters);

    appliedKp = kP;
    appliedKi = kI;
    appliedKd = kD;
    appliedKff = kFF;
}

  private void updatePidFromDashboard() {
    double kP = sbKp.getDouble(kDefaultKp);
    double kI = sbKi.getDouble(kDefaultKi);
    double kD = sbKd.getDouble(kDefaultKd);
    double kFF = sbKff.getDouble(kDefaultKff);

    if (kP != appliedKp || kI != appliedKi || kD != appliedKd || kFF != appliedKff) {
      applyPid(kP, kI, kD, kFF);
  }
}
@SuppressWarnings("removal")
  public void setVelocityRpm(double rpm) {
    motor.getClosedLoopController().setReference(rpm, ControlType.kVelocity);
  }

  public boolean isAtRpm(double targetRpm, double toleranceRpm) {
    return Math.abs(targetRpm - getMotorRpm()) <= toleranceRpm;
  }

  public void stop() {
    motor.setVoltage(0.0);
  }

  public Command runVolts(double volts) {
    return runEnd(() -> setVolts(volts), this::stop);
  }

  public Command runVolts(DoubleSupplier volts) {
    return runEnd(() -> setVolts(volts.getAsDouble()), this::stop);
  }

  public Command runPercent(double percent) {
    return runEnd(() -> setVolts(percent * 12.0), this::stop);
  }

  public Command runPercent(DoubleSupplier percent) {
    return runEnd(() -> setVolts(percent.getAsDouble() * 12.0), this::stop);
  }

  public Command runVelocityRpm(double rpm) {
  return runEnd(() -> setVelocityRpm(rpm), this::stop);
  }

  public Command runVelocityRpm(DoubleSupplier rpm) {
    return runEnd(() -> setVelocityRpm(rpm.getAsDouble()), this::stop);
  }


  public double getAppliedVolts() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  public double getFeedPercent() {
    if (sbFeedPercent != null) {
      return sbFeedPercent.getDouble(kFeedPercent);
    }
    return kFeedPercent; 
  }

  public double getReversePercent() {
    return sbReversePercent != null ? sbReversePercent.getDouble(kReverseFeedPercent):kReverseFeedPercent;
  }

  public double getManualVolts() {
    return sbManualVolts != null ? sbManualVolts.getDouble(kManualVolts):kManualVolts;
  }
  public double getFeedRpm() {
    return sbFeedRpm != null ? sbFeedRpm.getDouble(kFeedRPM):kFeedRPM;
  }

  public double getReverseRpm() {
    return sbReverseRpm != null ? sbReverseRpm.getDouble(kReverseFeedRPM):kReverseFeedRPM;
  }
}
