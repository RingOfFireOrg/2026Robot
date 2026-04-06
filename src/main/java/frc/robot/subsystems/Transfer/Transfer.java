package frc.robot.subsystems.Transfer;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

@SuppressWarnings("removal")
public class Transfer extends SubsystemBase {
  private static final int kMotorCanId = 31; //spindexer

  private final SparkMax motor = new SparkMax(kMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  private static final double kDeadband = 0.02;
  private static final double kMaxVolts = 12.0;
  private static final double kMinVoltsToMove = 1.5;
  private static final double kFeedPercent = -0.70;
  private static final double kReversePercent = 0.70;
  private static final double kManualVolts = -5.0;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Spindexer");

  private GenericEntry sbFeedPercent;
  private GenericEntry sbReversePercent;
  private GenericEntry sbManualVolts;

  public Transfer() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(40);

    motor.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    tuningInitialization();
  }

  // tuningIntialization adds ShuffleBoard widgets used for robot tuning
  private void tuningInitialization() {

    // Shuffleboard widget updates impact system performance as more variables are added.
    // Constants.tuningMode needs to be false unless you are tuning.
    if (Constants.tuningMode) {
      sbFeedPercent = tab.add("Feed %", kFeedPercent).getEntry();
      sbReversePercent = tab.add("Reverse %", kReversePercent).getEntry();
      sbManualVolts = tab.add("Manual Volts", kManualVolts).getEntry();

      tab.addNumber("RPM", encoder::getVelocity);
      tab.addNumber("Rotations", encoder::getPosition);
      tab.addNumber("Applied Volts", this::getAppliedVolts);
      tab.addNumber("Current", motor::getOutputCurrent);
      tab.addNumber("Motor Temp C", motor::getMotorTemperature);
    }
  }

  @Override
public void periodic() {
  if (getMotorRpm() < 50 && motor.getOutputCurrent() > 30) {
    runPercent(-0.3);
  }
}

  public void setVolts(double volts) {
    double cmd = MathUtil.applyDeadband(volts, kDeadband);
    cmd = MathUtil.clamp(cmd, -kMaxVolts, kMaxVolts);

    if (Math.abs(cmd) > 1e-6) {
      cmd = Math.copySign(Math.max(Math.abs(cmd), kMinVoltsToMove), cmd);
    }

    motor.setVoltage(cmd);
  }

  public void stop() {
    motor.setVoltage(0.0);
  }

  public Command runVolts(double volts) {
    return runEnd(() -> setVolts(volts), this::stop);
  }

  public Command runPercent(double percent) {
    return runEnd(() -> setVolts(percent * 12.0), this::stop);
  }
    public Command runVolts(DoubleSupplier volts) {
    return runEnd(() -> setVolts(volts.getAsDouble()), this::stop);
  }

  public Command runPercent(DoubleSupplier percent) {
    return runEnd(() -> setVolts(percent.getAsDouble() * 12.0), this::stop);
  }
    public double getAppliedVolts() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  public double getFeedPercent() {
    return sbFeedPercent != null ? sbFeedPercent.getDouble(kFeedPercent):kFeedPercent;
  }

  public double getMotorRotations() {
    return encoder.getPosition();
  }

  public double getMotorRpm() {
    return encoder.getVelocity();
  }

  public double getReversePercent() {
    return sbReversePercent != null ? sbReversePercent.getDouble(kReversePercent):kReversePercent;
  }

  public double getManualVolts() {
    return sbManualVolts != null ? sbManualVolts.getDouble(kManualVolts):kManualVolts;
  }
}
