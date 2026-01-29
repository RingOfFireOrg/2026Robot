package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.DriverStation;

public class LedManager {
  private final LedModeBus bus;

  private boolean shooterActive = false;
  private boolean climberActive = false;

  public LedManager(LedModeBus bus) {
    this.bus = bus;
  }

  public void setShooterActive(boolean active) {
    shooterActive = active;
  }

  public void setClimberActive(boolean active) {
    climberActive = active;
  }

  public void periodic() {
    if (DriverStation.isDisabled()) {
      bus.setMode(0);
      return;
    }

    if (climberActive) {
      bus.setMode(4);
      return;
    }

    if (shooterActive) {
      bus.setMode(3);
      return;
    }

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      bus.setMode(1);
    } else {
      bus.setMode(2);
    }
  }
}
