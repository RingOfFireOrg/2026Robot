package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double angularPosition = 0.0;

    public double elevatorPositionMeters = 0.0;

    public Pose3d endEffectorPose3d = new Pose3d();
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setVoltage(double volts) {}
  public default void setVoltageEject(double volts, double eject) {}


  public default void setAngle(double angle) {}
  public default void ejecter(double volts) {}
  public default void resetEncoder() {}

  public default void setAngleandIntake(double angle, double volts) {}


  public default void enableHoming(boolean enable) {}

}