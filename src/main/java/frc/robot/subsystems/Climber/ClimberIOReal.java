package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberIOReal implements ClimberIO {
  private static final int kLeftCanId = 50;
  private static final int kRightCanId = 51;

  private static final boolean kLeftInverted = false;
  private static final boolean kRightInverted = true;

  private final TalonFX left = new TalonFX(kLeftCanId);
  private final TalonFX right = new TalonFX(kRightCanId);

  private final VoltageOut voltsReq = new VoltageOut(0.0);

  public ClimberIOReal() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLimit = 40.0;
    cfg.CurrentLimits = limits;

    MotorOutputConfigs outLeft = new MotorOutputConfigs();
    outLeft.NeutralMode = NeutralModeValue.Brake;
    outLeft.Inverted = kLeftInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput = outLeft;
    left.getConfigurator().apply(cfg);

    MotorOutputConfigs outRight = new MotorOutputConfigs();
    outRight.NeutralMode = NeutralModeValue.Brake;
    outRight.Inverted = kRightInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput = outRight;
    right.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.connected = true;

    inputs.positionRot = left.getPosition().getValueAsDouble();
    inputs.velocityRps = left.getVelocity().getValueAsDouble();

    inputs.appliedVolts = left.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps =
        left.getSupplyCurrent().getValueAsDouble() + right.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    left.setControl(voltsReq.withOutput(volts));
    right.setControl(voltsReq.withOutput(volts));
  }
}
