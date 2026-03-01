package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class AlignToHub extends Command {
  private final Drive drive;
  private final Vision vision;
  private final int cameraIndex;

  private final PIDController xPid;
  private final PIDController yPid;
  private final ProfiledPIDController thetaPid;
  private final HolonomicDriveController controller;

  // how far away from april tag
  private static final double kStandoffMeters = 1.00;

  private static final double kMaxV = 1.5;     // m/s
  private static final double kMaxOmega = 2.5; // rad/s
  private static final double tag25Offset = -0.3;

  private double lastPrint = 0.0;

  public AlignToHub(Drive drive, Vision vision, int cameraIndex) {
    this.drive = drive;
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    addRequirements(drive);

    xPid = new PIDController(2.5, 0.0, 0.0);
    yPid = new PIDController(2.5, 0.0, 0.0);

    thetaPid =
        new ProfiledPIDController(
            5.0, 0.0, 0.15,
            new TrapezoidProfile.Constraints(6.0, 10.0));
    thetaPid.enableContinuousInput(-Math.PI, Math.PI);

    controller = new HolonomicDriveController(xPid, yPid, thetaPid);
  }

  @Override
  public void initialize() {
    xPid.reset();
    yPid.reset();
    thetaPid.reset(drive.getRotation().getRadians());
    lastPrint = 0.0;
    System.out.println("[AlignToHub] init cam=" + cameraIndex);
  }

  @Override
  public void execute() {
    if (!vision.hasTarget(cameraIndex)) {
      drive.stop();
      ratePrint("[AlignToHub] no target");
      return;
    }

    int tagId = (int) Math.round(vision.getTargetID(cameraIndex));
    
    var tagPoseOpt = aprilTagLayout.getTagPose(tagId);
    if (tagPoseOpt.isEmpty()) {
      drive.stop();
      ratePrint("[AlignToHub] unknown tag id=" + tagId);
      return;
    }
    Pose2d current = drive.getPose();
    Pose2d goal = drive.getPose();
    Pose2d tagPose = tagPoseOpt.get().toPose2d().transformBy(new Transform2d(-kStandoffMeters, 0.0, Rotation2d.fromDegrees(180.0)));

    //find a better way to do ts
    if(tagId == 26) goal = new Pose2d(tagPose.getX()-0.5, tagPose.getY(), tagPose.getRotation());
    if(tagId == 25) goal = new Pose2d(tagPose.getX()-0.5, tagPose.getY()+tag25Offset, tagPose.getRotation());

    

    ChassisSpeeds fieldRelative =
        controller.calculate(current, goal, 0.0, goal.getRotation());

    double vx = MathUtil.clamp(fieldRelative.vxMetersPerSecond, -kMaxV, kMaxV);
    double vy = MathUtil.clamp(fieldRelative.vyMetersPerSecond, -kMaxV, kMaxV);
    double omega = MathUtil.clamp(fieldRelative.omegaRadiansPerSecond, -kMaxOmega, kMaxOmega);
    
    
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drive.getRotation()));

    ratePrint(
        "[AlignToHub] id=" + tagId
            + " hasTarget=true"
            + " tx=" + String.format("%.2f", vision.getTxDeg(cameraIndex))
            + " cur=(" + fmt(current.getX()) + "," + fmt(current.getY()) + "," + fmtDeg(current.getRotation().getDegrees()) + ")"
            + " goal=(" + fmt(goal.getX()) + "," + fmt(goal.getY()) + "," + fmtDeg(goal.getRotation().getDegrees()) + ")"
            + " v=(" + fmt(vx) + "," + fmt(vy) + "," + fmt(omega) + ")"
    );
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    System.out.println("[AlignToHub] end interrupted=" + interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void ratePrint(String msg) {
    double now = Timer.getFPGATimestamp();
    if (now - lastPrint > 0.25) {
      System.out.println(msg);
      lastPrint = now;
    }
  }

  private static String fmt(double v) {
    return String.format("%.2f", v);
  }

  private static String fmtDeg(double v) {
    return String.format("%.1f", v);
  }
}
