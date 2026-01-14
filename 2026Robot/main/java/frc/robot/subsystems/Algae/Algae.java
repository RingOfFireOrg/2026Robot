package frc.robot.subsystems.Algae;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {
    private final AlgaeIO io;
    private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

    public Algae(AlgaeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Algae", inputs);
    }

    public Command runPercent(double percent) {
        return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
    }

    public Command runTeleop(DoubleSupplier input) {
        return run(() -> io.setVoltage((input.getAsDouble()) * 12.0));
    }
    public Command runTeleopIntake(DoubleSupplier input) {
        return run(() -> io.setVoltageIntake((input.getAsDouble()) * 12.0));
    }
    public Command runTeleopLaunch(DoubleSupplier left, DoubleSupplier right) {
        return run(() -> io.setVoltageLaunch((left.getAsDouble()) * 12.0, (right.getAsDouble()) * 12.0));
    }

    public Command runPosition(DoubleSupplier position) {
        return run(() -> io.setReference(position.getAsDouble()));
    }
    
    public Command runPositionandIntake(DoubleSupplier position, DoubleSupplier volts) {
        //return runOnce(() -> io.setAngleandIntake(angle, volts));
        return runOnce(() -> io.setVoltageIntake((volts.getAsDouble()) * 12.0))
        .andThen(run(() -> io.setReference(position.getAsDouble())));
    }
}