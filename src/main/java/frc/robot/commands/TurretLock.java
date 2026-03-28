// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.LimelightFrontName;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.MathUtil;
@SuppressWarnings("unused")

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretLock extends Command {
  private Turret turret;
  private PIDController turretPID;
 
  //private final double TURRET_P = 0.1;//change ts
  //private final double TURRET_D = 0;//change ts
 
  public TurretLock(Turret turret) {
    this.turret = turret;
    turretPID = new PIDController(turret.sbTURRET_P.getDouble(0.1), 0, turret.sbTURRET_D.getDouble(0));
    turretPID.setTolerance(1.0);
    addRequirements(turret);
  }
  private Double getTagTx(int wantedId) {
    LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(LimelightFrontName);
    for (LimelightHelpers.RawFiducial f : fiducials) {
      if (f.id == wantedId) {
        return f.txnc;
      }
    }
    return null;
  }

  private Double getMidpointTx(int id1, int id2) {
    Double tx1 = getTagTx(id1);
    Double tx2 = getTagTx(id2);

    if (tx1 == null || tx2 == null) {
      return null;
    }

    return (tx1 + tx2) / 2.0;
  }

  private double getSingleTagOffset(int id) {
    switch (id) {
      case 24:
      case 8:
        return turret.HubSideOffset.getDouble(1);

      case 25:
      case 9:
        return turret.HubMiddleOffset.getDouble(0.5);

      case 26:
      case 10:
        return 0.0;

      case 27:
      case 11:
        return -turret.HubSideOffset.getDouble(1);

      case 23:
      case 7:
        return turret.trenchOffset.getDouble(5);

      case 28:
      case 12:
        return -turret.trenchOffset.getDouble(5);

      default:
        return 0.0;
    }
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //turretPID.reset();
    turretPID.setP(turret.sbTURRET_P.getDouble(0.1));
    turretPID.setD(turret.sbTURRET_D.getDouble(0.0));
    turretPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*double offset = 0;
    switch ((int)LimelightHelpers.getFiducialID(LimelightFrontName)) {
      case 24:
        offset = turret.HubSideOffset.getDouble(1);//change ts
        break;
      case 25:
        offset = turret.HubMiddleOffset.getDouble(0.5);//change ts
        break;
      case 27:
        offset = -turret.HubSideOffset.getDouble(1);//change ts
        break;
      case 23: 
        offset = turret.trenchOffset.getDouble(5);//change ts
        break;
      case 28: 
        offset = -turret.trenchOffset.getDouble(5);//change ts;
        break;
      case 8:
        offset = turret.HubSideOffset.getDouble(1);//change ts
        break;
      case 9:
        offset = turret.HubMiddleOffset.getDouble(0.5);//change ts
        break;
      case 11:
        offset = -turret.HubSideOffset.getDouble(1);//change ts
        break;
      case 7: 
        offset = turret.trenchOffset.getDouble(5);//change ts
        break;
      case 12: 
        offset = -turret.trenchOffset.getDouble(5);//change ts;
        break;
      default:
        break;
    }
    double tx = LimelightHelpers.getTX(LimelightFrontName);

    if (LimelightHelpers.getTV(LimelightFrontName)
    && (int) LimelightHelpers.getFiducialID(LimelightFrontName) == 26) {
      //double output = turretPID.calculate(
          //LimelightHelpers.getTX(LimelightFrontName));
      double output = turretPID.calculate(tx - offset, 0.0);
      System.out.println("tx" + tx + "Turret PID output: " + output);
      //if (output > 1) output = 1;
      //if (output < -1) output = -1;
      output = MathUtil.clamp(output, -0.20, 0.20);
      if (Math.abs(tx - offset) < 1.0) {
        turret.stopTurret();
        return;
      }
      turret.setDutyCycle(output); 
    } else {
      turret.stopTurret();
    }
  }*/
  if (!LimelightHelpers.getTV(LimelightFrontName)) {
    turretPID.reset();
    turret.stopTurret();
    return;
  }

    Double desiredTx = null;

    Double mid25_26 = getMidpointTx(25, 26);
    Double mid26_27 = getMidpointTx(26, 27);
    Double mid24_25 = getMidpointTx(24, 25);

    Double mid9_10 = getMidpointTx(9, 10);
    Double mid10_11 = getMidpointTx(10, 11);
    Double mid8_9 = getMidpointTx(8, 9);

    if (mid25_26 != null) {
      desiredTx = mid25_26;
    } else if (mid26_27 != null) {
      desiredTx = mid26_27;
    } else if (mid24_25 != null) {
      desiredTx = mid24_25;
    } else if (mid9_10 != null) {
      desiredTx = mid9_10;
    } else if (mid10_11 != null) {
      desiredTx = mid10_11;
    } else if (mid8_9 != null) {
      desiredTx = mid8_9;
    } else {
      int id = (int) LimelightHelpers.getFiducialID(LimelightFrontName);
      double tx = LimelightHelpers.getTX(LimelightFrontName);
      double offset = getSingleTagOffset(id);
      desiredTx = tx - offset;
    }

    double output = turretPID.calculate(desiredTx);
    output = MathUtil.clamp(output, -0.20, 0.20);

    System.out.println("desiredTx=" + desiredTx + " output=" + output);

    if (Math.abs(desiredTx) < 5.0) {
      turretPID.reset();
      turret.stopTurret();
      return;
    }
    if (Math.abs(output) < 0.1) {
      turretPID.reset();
      turret.stopTurret();
      return;
    }
    turret.setDutyCycle(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
