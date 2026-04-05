package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.LimelightFrontName;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.LimelightHelpers;

public class Shuttle extends Command {
  private final Turret turret;
  private final Indexer indexer;
  private final Transfer transfer;

  private final PIDController turretPid = new PIDController(0.10, 0.0, 0.0);

  private static final double kMaxTurretDuty = 0.20;
  private static final double kTurretToleranceDeg = 1.2;

  private static final double kTopRpm = 4500.0; //change ts
  private static final double kBottomRpm = 3500.0;

  private static final double kTopRpmTol = 120.0;
  private static final double kBottomRpmTol = 120.0;

  public Shuttle(Turret turret, Indexer indexer, Transfer transfer) {
    this.turret = turret;
    this.indexer = indexer;
    this.transfer = transfer;

    addRequirements(turret, indexer, transfer);
  }

  @Override
  public void initialize() {
    turretPid.reset();
    turretPid.setP(turret.sbTurretKp.getDouble(0.10));
    turretPid.setD(turret.sbTurretKd.getDouble(0.0));
  }

  @Override
  public void execute() {
    turretPid.setP(turret.sbTurretKp.getDouble(0.10));
    turretPid.setD(turret.sbTurretKd.getDouble(0.0));

    turret.setShooterRPM(kTopRpm, kBottomRpm);

    if (!hasValidShuttleTag()) {
      turret.stopTurret();
      indexer.stop();
      transfer.stop();
      return;
    }

    double tx = LimelightHelpers.getTX(LimelightFrontName);
    double desiredTx = getShuttleOffsetDeg();

    double turretOutput = turretPid.calculate(tx, desiredTx);
    turretOutput = MathUtil.clamp(turretOutput , -kMaxTurretDuty, kMaxTurretDuty);
    turret.setDutyCycle(turretOutput); //might need a -

    boolean aimed = Math.abs(tx - desiredTx) <= kTurretToleranceDeg;
    boolean shooterReady =
        Math.abs(turret.getShooterTopMeasuredRpm() - kTopRpm) <= kTopRpmTol
            && Math.abs(turret.getShooterBottomMeasuredRpm() - kBottomRpm) <= kBottomRpmTol;

    if (aimed && shooterReady) {
      indexer.runPercent(-0.8);
      transfer.runPercent(-0.8);
    } else {
      indexer.stop();
      transfer.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopTurret();
    turret.stopShooter();
    indexer.stop();
    transfer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean hasValidShuttleTag() {
    if (!LimelightHelpers.getTV(LimelightFrontName)) {
      return false;
    }

    int id = (int) LimelightHelpers.getFiducialID(LimelightFrontName);

    return id == 6 || id == 5 || id == 4 || id == 3 || id == 2 || id == 1 ||
          id == 17 || id == 18 || id == 19 || id == 20 || id == 21 || id == 22;
  }

  private double getShuttleOffsetDeg() {
    int id = (int) LimelightHelpers.getFiducialID(LimelightFrontName);

    switch (id) {//change te offsets
      case 6:
        return 2.5;
      case 5:
        return 1.5;
      case 4:
        return 0.5;
      case 3:
        return -0.5;
      case 2:
        return -1.5;
      case 1:
        return -2.5;

      case 17:
        return -2.5;
      case 18:
        return -1.5;
      case 19:
        return -0.5;
      case 20:
        return 0.5;
      case 21:
        return 1.5;
      case 22:
        return 2.5;


      default:
        return 0.0;
    }
  }
}