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
        return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
    }

    public Command runTeleop(DoubleSupplier input) {
        return run(() -> io.setVoltage((input.getAsDouble()) * 12.0));
    }

    public Command runLauncher(double percent) {
        return runEnd(() -> io.moveLauncher(percent * 12.0), () -> io.moveLauncher(0.0));
    }

    public Command autoLaunch() {
        return run(() -> io.moveLauncher(4.6))
        .until(() -> io.checkLauncher())
        .finallyDo(() -> io.moveLauncher(0));
    }
}