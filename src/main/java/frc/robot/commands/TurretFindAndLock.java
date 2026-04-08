package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.LimelightFrontName;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.LimelightHelpers;

public class TurretFindAndLock extends Command {
  private final Turret turret;
  private final TurretLock turretLock;

  private boolean locked = false;
  private double sweepDir = 1.0;

  private static final double kSweepDuty = 0.12;
  private static final double kSearchMinAngleDeg = -85.0; //change to 90 after testing, to make sure wires are fine
  private static final double kSearchMaxAngleDeg = 85.0;

  public TurretFindAndLock(Turret turret) {
    this.turret = turret;
    this.turretLock = new TurretLock(turret);
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    locked = false;
    turretLock.initialize();

    double angle = turret.getTurretAngleDeg();
    if (angle >= kSearchMaxAngleDeg) {
      sweepDir = -1.0;
    } else if (angle <= kSearchMinAngleDeg) {
      sweepDir = 1.0;
    }
  }

  @Override
  public void execute() {
    if (!locked) {
      if(!LimelightHelpers.getTV(LimelightFrontName)) {
        double angle = turret.getTurretAngleDeg();

        if (angle >= kSearchMaxAngleDeg) {
          sweepDir = -1.0;
        } else if (angle <= kSearchMinAngleDeg) {
          sweepDir = 1.0;
        }

        turret.setDutyCycle(sweepDir * kSweepDuty);
        return;
      }
    }
    turretLock.execute();
  }

  @Override
  public void end(boolean interrupted) {
    turretLock.end(interrupted);
    turret.stopTurret();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean hasValidHubTag() {
    if (!LimelightHelpers.getTV(LimelightFrontName)) {
      return false;
    }

    int id = (int) LimelightHelpers.getFiducialID(LimelightFrontName);
    return id == 9 || id == 10 || id == 8 || id == 11 || id == 24
        || id == 25 || id == 26 || id == 27;
  }
}