package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;

public class HubTurn extends Command {
  private final Drive drive;
  private final String ll;

  private final PIDController pid = new PIDController(0.02, 0.0, 0.001);

  private static final double kTxTolDeg = 1.0;
  private static final double kMaxOmegaRadPerSec = 3.5;
  private static final double kMinOmegaRadPerSec = 0.25;

  private static final double kOmegaSign = -1.0; //change if turning the wrong way

  public HubTurn(Drive drive, String limelightName) {
    this.drive = drive;
    this.ll = limelightName;
    addRequirements(drive);
  }

  private static LimelightTarget_Fiducial bestTag(String ll) {
  LimelightResults res = LimelightHelpers.getLatestResults(ll);
  if (res == null || res.targets_Fiducials == null || res.targets_Fiducials.length == 0) return null;

  LimelightTarget_Fiducial best = null;
  double bestArea = -1.0;
  double bestAbsTx = 1e9;

  for (LimelightTarget_Fiducial t : res.targets_Fiducials) {
    if (t == null) continue;

    double area = t.ta;
    double absTx = Math.abs(t.tx);

    if (area > bestArea || (Math.abs(area - bestArea) < 1e-9 && absTx < bestAbsTx)) {
      bestArea = area;
      bestAbsTx = absTx;
      best = t;
    }
  }

  return best;
}

  @Override
  public void initialize() {
    pid.reset();
  }

  @Override
  public void execute() {
    if (!LimelightHelpers.getTV(ll)) {
      drive.runVelocity(new ChassisSpeeds());
      return;
    }

    LimelightTarget_Fiducial t = bestTag(ll);
    if (t == null) {
      drive.runVelocity(new ChassisSpeeds());
      return;
    }

    double txDeg = t.tx;

    double omega = pid.calculate(txDeg, 0.0);
    omega = kOmegaSign * omega;
    omega = MathUtil.clamp(omega, -kMaxOmegaRadPerSec, kMaxOmegaRadPerSec);

    if (Math.abs(txDeg) > kTxTolDeg) {
      omega = Math.copySign(Math.max(Math.abs(omega), kMinOmegaRadPerSec), omega);
    } else {
      omega = 0.0;
    }

    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    LimelightTarget_Fiducial t = bestTag(ll);
    if (t == null) return false;
    return Math.abs(t.tx) <= kTxTolDeg;
  }
}