package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class PhotonAlign extends Command {
  private final Drive drive;
  private final Vision vision;
  private final int cameraIndex;

  private final PIDController thetaPid;

  private static final double kMaxOmega = 2.5;
  private static final double kStopDeg = 1.5;

  private double lastPrint = 0.0;

  public PhotonAlign(Drive drive, Vision vision, int cameraIndex) {
    this.drive = drive;
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    addRequirements(drive);

    thetaPid = new PIDController(4.0, 0.0, 0.2);
    thetaPid.setTolerance(Math.toRadians(kStopDeg));
  }

  @Override
  public void initialize() {
    thetaPid.reset();
    lastPrint = 0.0;
    System.out.println("[PhotonAlign] init cam=" + cameraIndex);
  }

  @Override
  public void execute() {
    if (!vision.hasTarget(cameraIndex)) {
      drive.stop();
      ratePrint("[PhotonAlign] no target");
      return;
    }

    double txDeg = vision.getTxDeg(cameraIndex);
    double errorRad = Math.toRadians(txDeg);

    double omega = thetaPid.calculate(errorRad, 0.0);
    omega = MathUtil.clamp(omega, -kMaxOmega, kMaxOmega);

    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));

    ratePrint(
        "[PhotonAlign] cam=" + cameraIndex
            + " id=" + (int) Math.round(vision.getTargetID(cameraIndex))
            + " txDeg=" + String.format("%.2f", txDeg)
            + " omega=" + String.format("%.2f", omega));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    System.out.println("[PhotonAlign] end interrupted=" + interrupted);
  }

  @Override
  public boolean isFinished() {
    return vision.hasTarget(cameraIndex) && thetaPid.atSetpoint();
  }

  private void ratePrint(String msg) {
    double now = Timer.getFPGATimestamp();
    if (now - lastPrint > 0.25) {
      System.out.println(msg);
      lastPrint = now;
    }
  }
}
