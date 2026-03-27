package frc.robot.subsystems.Climber;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.FeedbackSensor;
@SuppressWarnings("removal")

public class Climber extends SubsystemBase {
  private static final int kMotorCanId = 50;

  private final SparkMax motor = new SparkMax(kMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  private static final double kDeadband = 0.02;
  private static final double kMaxVolts = 8.0;
  private static final double kMinVoltsToMove = 1.5;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Climber");

  private final GenericEntry sbTopHeight =
    tab.add("Top Height", 185).getEntry();

  private final GenericEntry sbBottomHeight =
    tab.add("Bottom Height", 0).getEntry();

  private final GenericEntry sbPosition =
    tab.add("Climber Position", 0).getEntry();
  private final SparkClosedLoopController controller = motor.getClosedLoopController();

  public Climber() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.smartCurrentLimit(40);
    config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.2, 0.0, 0.0);

    motor.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    encoder.setPosition(0.0);
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

    double pos = encoder.getPosition();
    double top = sbTopHeight.getDouble(185);
    double bottom = sbBottomHeight.getDouble(0);

    if (cmd > 0 && pos >= top) cmd = 0;
    if (cmd < 0 && pos <= bottom) cmd = 0;

    if (Math.abs(cmd) > 1e-6) {
      cmd = Math.copySign(Math.max(Math.abs(cmd), kMinVoltsToMove), cmd);
    }

    motor.setVoltage(cmd);
  }

  public void setPosition(double targetRotations) {
    double top = sbTopHeight.getDouble(185);
    double bottom = sbBottomHeight.getDouble(0);
    double clampedTarget = MathUtil.clamp(targetRotations, bottom, top);
    controller.setSetpoint(clampedTarget, ControlType.kPosition);
  }
  public Command goTop() {
    return runOnce(() -> setPosition(sbTopHeight.getDouble(185)));
  }

  public Command goBottom() {
    return runOnce(() -> setPosition(sbBottomHeight.getDouble(0)));
  }

  public void stop() {
    motor.setVoltage(0.0);
  }

  public Command runTeleop(DoubleSupplier percent) {
    return runEnd(() -> runVolts(percent.getAsDouble() * 12.0), ()->stop());
  }
  public Command runVolts(double volts) {
    return runEnd(() -> setVolts(volts), this::stop);
  }

  public Command runPercent(double percent) {
    return runEnd(() -> setVolts(percent * 12.0), this::stop);
  }
@Override
public void periodic() {
  sbPosition.setDouble(encoder.getPosition());
}
}

