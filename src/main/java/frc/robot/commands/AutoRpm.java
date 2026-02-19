package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.LimelightHelpers;

public class AutoRpm extends Command {
  private final Turret turret;
  private final String limelightName;
  private double lastPrint = 0.0;

  public AutoRpm(Turret turret, String limelightName) {
    this.turret = turret;
    this.limelightName = limelightName;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    if (!LimelightHelpers.getTV(limelightName)) {
      turret.stopShooter();
      return;
    }

    double distMeters = turret.getDistanceToTagMeters(limelightName);
    turret.setShooterFromDistanceMeters(distMeters);

    double now = Timer.getFPGATimestamp();
    if (now - lastPrint > 0.25) {
      double rpmSet = turret.getShooterRpmForDistanceMeters(distMeters);
      System.out.println(
          "[AutoRpm] dist=" + String.format("%.2f", distMeters)
              + " rpmSet=" + String.format("%.0f", rpmSet));
      lastPrint = now;
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopShooter();
    System.out.println("[AutoRpm] end interrupted=" + interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}