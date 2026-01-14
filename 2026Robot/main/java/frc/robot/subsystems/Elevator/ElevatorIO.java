package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double controllerSetpoint = 0.0;
    public double controllerOutput = 0.0;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double angularPositionRot = 0.0;
    public double elevatorPositionMeters = 0.0;
    public Pose3d elevatorPose3d = new Pose3d();

  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default double getHeight(double height) {return height;}
  public default void setHeight(double height) {}
  public default void zeroElevator() {}


}