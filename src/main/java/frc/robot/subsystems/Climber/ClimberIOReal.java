package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberIOReal implements ClimberIO {
  private static final int kClimberCanId = 50;
  private static final boolean kInverted = false;

  private final TalonFX motor = new TalonFX(kClimberCanId);
  private final VoltageOut voltsReq = new VoltageOut(0.0);

  public ClimberIOReal() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLimit = 40.0;
    cfg.CurrentLimits = limits;

    MotorOutputConfigs out = new MotorOutputConfigs();
    out.NeutralMode = NeutralModeValue.Brake;
    out.Inverted = kInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput = out;

    motor.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.connected = true;

    inputs.positionRot = motor.getPosition().getValueAsDouble();
    inputs.velocityRps = motor.getVelocity().getValueAsDouble();

    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltsReq.withOutput(volts));
  }
}
