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
  private static final int kBottomMotorCanId = 30;
  private static final int kTopMotorCanId = 31;

  private final SparkMax leftMotor = new SparkMax(kBottomMotorCanId, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(kTopMotorCanId, MotorType.kBrushless);

  private static final double kDeadband = 0.02;
  private static final double kMaxVolts = 8.0;
  private static final double kMinVoltsToMove = 1.5;

public Transfer() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(30);

    leftMotor.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    SparkMaxConfig invertedConfig = new SparkMaxConfig();
    invertedConfig.apply(config);
    invertedConfig.inverted(true);

    rightMotor.configure(
        invertedConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  public void setVolts(double volts) {
    double cmd = MathUtil.applyDeadband(volts, kDeadband);
    cmd = MathUtil.clamp(cmd, -kMaxVolts, kMaxVolts);

    if (Math.abs(cmd) > 1e-6) {
      cmd = Math.copySign(Math.max(Math.abs(cmd), kMinVoltsToMove), cmd);
    }

    leftMotor.setVoltage(cmd);
    rightMotor.setVoltage(cmd);
  }

  public void stop() {
    leftMotor.setVoltage(0.0);
    rightMotor.setVoltage(0.0);
  }

  public Command runVolts(double volts) {
    return runEnd(() -> setVolts(volts), this::stop);
  }

  public Command runPercent(double percent) {
    return runEnd(() -> setVolts(percent * 12.0), this::stop);
  }
}
