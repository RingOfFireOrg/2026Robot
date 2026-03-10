package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {

    private final Pigeon2 pigeon;

    public GyroIOPigeon2(int id, String canBus) {
        pigeon = new Pigeon2(id, canBus);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {

        inputs.connected = pigeon.isConnected();

        inputs.yawPosition =
                Rotation2d.fromDegrees(-pigeon.getYaw().getValueAsDouble());

        inputs.yawVelocityRadPerSec =
                Units.degreesToRadians(
                        -pigeon.getAngularVelocityZWorld().getValueAsDouble());
    }
}