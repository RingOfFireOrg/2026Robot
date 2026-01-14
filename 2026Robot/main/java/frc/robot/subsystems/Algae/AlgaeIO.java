package frc.robot.subsystems.Algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
  @AutoLog
  public static class AlgaeIOInputs {
    public double controllerSetpoint = 0.0;
    public double controllerOutput = 0.0;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double angularPositionRot = 0.0;


    public boolean atGoal = false;
  }

  public default void updateInputs(AlgaeIOInputs inputs) {}

  public default void setVoltage(double volts) {}
  public default void setVoltageIntake(double volts) {}
  public default void setVoltageLaunch(double left, double right) {}

  public default void setPosition(double position) {}
  public default void setReference(double position) {}

}