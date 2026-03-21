package frc.robot.subsystems.Transfer;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

@SuppressWarnings("removal")
public class Transfer extends SubsystemBase {
  private static final int kMotorCanId = 31;//spindexer

  private final SparkMax motor = new SparkMax(kMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private static final double kDeadband = 0.02;
  private static final double kMaxVolts = 12.0;
  private static final double kMinVoltsToMove = 1.5;
  private final ShuffleboardTab tab = Shuffleboard.getTab("Spindexer");

  private final GenericEntry sbFeedPercent = tab.add("Feed %", -0.70).getEntry();
  private final GenericEntry sbReversePercent = tab.add("Reverse %", 0.70).getEntry();
  private final GenericEntry sbManualVolts = tab.add("Manual Volts", -5.0).getEntry();
  private final GenericEntry sbMaxVolts = tab.add("Clamp Max Volts", kMaxVolts).getEntry();
  private final GenericEntry sbMinMoveVolts = tab.add("Min Move Volts", kMinVoltsToMove).getEntry();

  public Transfer() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(40);

    motor.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  
    tab.addNumber("RPM", encoder::getVelocity);
    tab.addNumber("Rotations", encoder::getPosition);
    tab.addNumber("Applied Volts", this::getAppliedVolts);
    tab.addNumber("Current", motor::getOutputCurrent);
    tab.addNumber("Motor Temp C", motor::getMotorTemperature);
  }

  public void setVolts(double volts) {
    double maxVolts = Math.abs(sbMaxVolts.getDouble(kMaxVolts));
    double minMoveVolts = Math.abs(sbMinMoveVolts.getDouble(kMinVoltsToMove));

    double cmd = MathUtil.applyDeadband(volts, kDeadband);
    cmd = MathUtil.clamp(cmd, -maxVolts, maxVolts);

    if (Math.abs(cmd) > 1e-6) {
      cmd = Math.copySign(Math.max(Math.abs(cmd), minMoveVolts), cmd);
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
    return sbFeedPercent.getDouble(-0.60);
  }

  public double getReversePercent() {
    return sbReversePercent.getDouble(0.60);
  }

  public double getManualVolts() {
    return sbManualVolts.getDouble(-5.0);
  }
}
