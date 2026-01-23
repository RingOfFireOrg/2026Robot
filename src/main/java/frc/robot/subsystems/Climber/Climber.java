package frc.robot.subsystems.Climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }
  public Command runPercent(double percent) {
  return runEnd(() -> io.setVoltage(percent * 12.0), io::stop);
}


  public Command runTeleop(DoubleSupplier percent) {
    return runEnd(() -> io.setVoltage(percent.getAsDouble() * 12.0), io::stop);
  }

  public Command runVolts(DoubleSupplier volts) {
    return runEnd(() -> io.setVoltage(volts.getAsDouble()), io::stop);
  }

  public Command stopCommand() {
    return runOnce(io::stop);
  }
}
