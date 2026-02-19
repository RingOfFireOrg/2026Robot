package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LimelightHelpers;

public class HubLock extends Command {
  private final Turret turret;
  private final Vision vision;
  private final int cameraIndex;

  private boolean wrapping = false;
  private double wrapTargetRot = 0.0;

  private static final double kP = 0.02;
  private static final double kMaxDuty = 0.30;
  private static final double kMinDuty = 0.09;
  private static final double kDeadbandDeg = 0.7;

  private static final double MAX_ROT = 0.95;
  private static final double WRAP_TRIGGER_ROT = 0.15;

  private static final double WRAP_KP = 1.2;
  private static final double WRAP_MAX_DUTY = 0.35;
  private static final double WRAP_DONE_TOL_ROT = 0.01;

  private double lastPrint = 0.0;

  public HubLock(Turret turret, Vision vision, int cameraIndex) {
    this.turret = turret;
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    double turretRot = turret.getMotorRotations();

    if (wrapping) {
      double err = wrapTargetRot - turretRot;
      double duty = MathUtil.clamp(err * WRAP_KP, -WRAP_MAX_DUTY, WRAP_MAX_DUTY);
      turret.setDutyCycle(duty);

      if (Math.abs(err) <= WRAP_DONE_TOL_ROT) {
        wrapping = false;
        turret.stop();
      }
      return;
    }

    boolean hasTarget = vision.hasTarget(cameraIndex);
    if (!hasTarget) {
      turret.stop();
      //turret.stopShooter();
      return;
    }

    double txDeg = vision.getTxDeg(cameraIndex);

    double neededRot = -txDeg / 360.0;
    double wouldBeRot = turretRot + neededRot;

    boolean wouldExceed = (wouldBeRot > MAX_ROT) || (wouldBeRot < -MAX_ROT);

    if (wouldExceed && Math.abs(neededRot) <= WRAP_TRIGGER_ROT) {
      double wrappedNeededRot = neededRot - Math.copySign(1.0, neededRot);
      wrapTargetRot = turretRot + wrappedNeededRot;
      wrapping = true;
      return;
    }

    double err = MathUtil.applyDeadband(txDeg, kDeadbandDeg);
    double duty = kP * err;
    duty = MathUtil.clamp(duty, -kMaxDuty, kMaxDuty);
    if (Math.abs(duty) > 1e-6) {
      duty = Math.copySign(Math.max(Math.abs(duty), kMinDuty), duty);
    }

    double distMeters =
        LimelightHelpers.getTargetPose3d_CameraSpace("limelight-tag")
            .getTranslation()
            .getNorm();

    //turret.setShooterFromDistanceMeters(distMeters);
    turret.setDutyCycle(duty);

    double now = Timer.getFPGATimestamp();
    if (now - lastPrint > 0.25) {
      //double rpmSet = turret.getShooterRpmForDistanceMeters(distMeters);
      System.out.println(
          "[HubLock] hasTarget=" + hasTarget
              + " txDeg=" + String.format("%.2f", txDeg)
              + " duty=" + String.format("%.3f", duty)
              + " dist=" + String.format("%.2f", distMeters)
              //+ " rpmSet=" + String.format("%.0f", rpmSet)
              + " turretRot=" + String.format("%.3f", turretRot)
              + " wrapping=" + wrapping);
      lastPrint = now;
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
    //turret.stopShooter();
    System.out.println("[HubLock] end interrupted=" + interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

/* 
public class HubLock extends Command {
  private final Turret turret;
  private final Vision vision;
  private final int cameraIndex;
  private boolean wrapping = false;
  private double wrapTargetRot = 0.0;


  private static final double kP = 0.02;        
  private static final double kMaxDuty = 0.30;
  private static final double kMinDuty = 0.09;  
  private static final double kDeadbandDeg = 0.7;
  private static final double MAX_ROT = 0.95;
  private static final double WRAP_TRIGGER_ROT = 0.15;

  private static final double WRAP_KP = 1.2;
  private static final double WRAP_MAX_DUTY = 0.35;
  private static final double WRAP_DONE_TOL_ROT = 0.01;


  private double lastPrint = 0.0;

  public HubLock(Turret turret, Vision vision, int cameraIndex) {
    this.turret = turret;
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    addRequirements(turret);
  }

  @Override
public void execute() {
  double turretRot = turret.getMotorRotations();

  if (wrapping) {
    double err = wrapTargetRot - turretRot;
    double duty = MathUtil.clamp(err * WRAP_KP, -WRAP_MAX_DUTY, WRAP_MAX_DUTY);
    turret.setDutyCycle(duty);

    if (Math.abs(err) <= WRAP_DONE_TOL_ROT) {
      wrapping = false;
      turret.stop();
    }
    return;
  }

  boolean hasTarget = vision.hasTarget(cameraIndex);
  if (!hasTarget) {
    turret.stop();
    turret.enableAngler(false);
    return;
  }

  double txDeg = vision.getTxDeg(cameraIndex);

  double neededRot = -txDeg / 360.0;
  double wouldBeRot = turretRot + neededRot;

  boolean wouldExceed = (wouldBeRot > MAX_ROT) || (wouldBeRot < -MAX_ROT);

  if (wouldExceed && Math.abs(neededRot) <= WRAP_TRIGGER_ROT) {
    double wrappedNeededRot = neededRot - Math.copySign(1.0, neededRot);
    wrapTargetRot = turretRot + wrappedNeededRot;
    wrapping = true;
    return;
  }

  double duty = 0.0;

  double err = MathUtil.applyDeadband(txDeg, kDeadbandDeg);
  duty = kP * err;
  duty = MathUtil.clamp(duty, -kMaxDuty, kMaxDuty);
  if (Math.abs(duty) > 1e-6) {
    duty = Math.copySign(Math.max(Math.abs(duty), kMinDuty), duty);
  }

  double distMeters =
      LimelightHelpers.getTargetPose3d_CameraSpace("limelight-tag")
          .getTranslation()
          .getNorm();
  turret.enableAngler(true);
  turret.setAnglerDistanceMeters(distMeters);

  turret.setDutyCycle(duty);

  double now = Timer.getFPGATimestamp();
  if (now - lastPrint > 0.25) {
    System.out.println(
        "[HubLock] hasTarget=" + hasTarget
            + " txDeg=" + String.format("%.2f", txDeg)
            + " duty=" + String.format("%.3f", duty)
            + " turretRot=" + String.format("%.3f", turretRot)
            + " wrapping=" + wrapping);
    lastPrint = now;
  }
}
*/
/* 
  @Override
  public void execute() {


    
    boolean hasTarget = vision.hasTarget(cameraIndex);
    double txDeg = vision.getTxDeg(cameraIndex);
    

    double duty = 0.0;
    try {
    if (hasTarget) {
      double err = MathUtil.applyDeadband(txDeg, kDeadbandDeg);

      //inverter
      duty = kP * err;

      duty = MathUtil.clamp(duty, -kMaxDuty, kMaxDuty);
      if (Math.abs(duty) > 1e-6) {
        duty = Math.copySign(Math.max(Math.abs(duty), kMinDuty), duty);
      }
    
    double distMeters = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-tag")
      .getTranslation()
      .getNorm();
    turret.enableAngler(true);
    turret.setAnglerDistanceMeters(distMeters);
    } else {

    turret.enableAngler(false);
  }
  
   

    turret.setDutyCycle(duty);

    double now = Timer.getFPGATimestamp();
    if (now - lastPrint > 0.25) {
      System.out.println("[HubLock] hasTarget=" + hasTarget
          + " txDeg=" + String.format("%.2f", txDeg)
          + " duty=" + String.format("%.3f", duty));
      lastPrint = now;
    }
  } catch (Exception e) {
      System.out.println("hi");}
      
    }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
    turret.enableAngler(false);
    System.out.println("[HubLock] end interrupted=" + interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}





/*package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.LimelightHelpers;

public class HubLock extends Command {
  private static final String LL = "limelight";
  private static final int PIPELINE = 0;

  private static final Set<Integer> HUB_TAG_IDS = Set.of(2, 3, 4, 5, 8, 9, 10, 11);

  private final Turret turret;
  private final PIDController pid = new PIDController(0.02, 0.0, 0.001);

  private int lockedId = -1;

  public HubLock(Turret turret) {
    this.turret = turret;
    addRequirements(turret);

    pid.setSetpoint(0.0);
    pid.setTolerance(1.0);
  }

  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex(LL, PIPELINE);
    pid.reset();
    lockedId = -1;
  }

  @Override
  public void execute() {
    boolean tv = LimelightHelpers.getTV(LL);
    int id = (int) LimelightHelpers.getFiducialID(LL);

    if (!tv) {
      pid.reset();
      turret.stop();
      return;
    }

    if (!HUB_TAG_IDS.contains(id)) {
      pid.reset();
      turret.stop();
      return;
    }

    if (lockedId == -1) lockedId = id;
    if (id != lockedId) {
      pid.reset();
      turret.stop();
      
      return;
    }

    double[] targetSpace = LimelightHelpers.getBotPose_TargetSpace(LL);
    double yawErrorDeg = targetSpace[4]; 
    System.out.println("HubLock | id=" + id + " yawErr=" + yawErrorDeg);

    double out = pid.calculate(yawErrorDeg);

    out = MathUtil.clamp(out, -0.25, 0.25);
    turret.setDutyCycle(out);
  }

  @Override
  public void end(boolean interrupted) {
    pid.reset();
    turret.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
*/