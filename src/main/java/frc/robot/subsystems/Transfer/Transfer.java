package frc.robot.subsystems.Transfer;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("removal")
public class Transfer extends SubsystemBase {
  private static final int kMotorCanId = 30;//spindexer

  private final SparkMax motor = new SparkMax(kMotorCanId, MotorType.kBrushless);

  private static final double kDeadband = 0.02;
  private static final double kMaxVolts = 12.0;
  private static final double kMinVoltsToMove = 1.5;

  public Transfer() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(40);

    motor.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
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
}
