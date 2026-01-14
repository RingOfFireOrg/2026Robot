package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LoggedTunableNumber;

public class AlignToReef extends Command {
  private PIDController xController, yController, rotController;
  @SuppressWarnings("unused")
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private Drive drive;
  private double tagID = -1;
  
  public enum reefSide {
    CENTER,
    LEFT,
    RIGHT
  }
  private reefSide side;
  private LoggedTunableNumber xP = new LoggedTunableNumber("Align/xP", Constants.X_REEF_ALIGNMENT_P);
  private LoggedTunableNumber yP = new LoggedTunableNumber("Align/yP", Constants.Y_REEF_ALIGNMENT_P);
  private LoggedTunableNumber zP = new LoggedTunableNumber("Align/zP", Constants.ROT_REEF_ALIGNMENT_P);

  public AlignToReef(Drive drive, reefSide side) {

    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.side = side;
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight-tag", 0);
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    xController = new PIDController(xP.getAsDouble(), 0.0, 0);  // Vertical movement
    yController = new PIDController(yP.getAsDouble(), 0.0, 0);  // Horitontal movement
    rotController = new PIDController(zP.getAsDouble(), 0, 0);  // Rotation


    rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

    if (side == reefSide.CENTER) {
      yController.setSetpoint(0);
    }
    else if (side == reefSide.RIGHT) {
      yController.setSetpoint(Constants.Y_SETPOINT_REEF_ALIGNMENT);

    }
    else if (side == reefSide.LEFT) {
      yController.setSetpoint(-Constants.Y_SETPOINT_REEF_ALIGNMENT);
    }
    else {
      yController.setSetpoint(0);

    }
    yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);
    tagID = LimelightHelpers.getFiducialID("limelight-tag");
  }

  @Override
  public void execute() {
    LoggedTunableNumber.ifChanged(
      hashCode(), 
      () -> {        
        xController = new PIDController(xP.getAsDouble(), 0.0, 0);  // Vertical movement
        yController = new PIDController(yP.getAsDouble(), 0.0, 0);  // Horitontal movement
        rotController = new PIDController(zP.getAsDouble(), 0, 0);  // Rotation

      }, 
      xP, yP, zP
    );

    if (LimelightHelpers.getTV("limelight-tag") && LimelightHelpers.getFiducialID("limelight-tag") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-tag");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      ChassisSpeeds speeds = new ChassisSpeeds(
        xSpeed * drive.getMaxLinearSpeedMetersPerSec() *0,
        ySpeed * drive.getMaxLinearSpeedMetersPerSec()/1.7,
        rotValue * drive.getMaxAngularSpeedRadPerSec()/2
      );
      boolean isFlipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
      speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, isFlipped ? new Rotation2d() : new Rotation2d());
      drive.runVelocity(speeds);

      //drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);


      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } 
    else {
      ChassisSpeeds speeds = new ChassisSpeeds(0,0,0);
      drive.runVelocity(speeds);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds speeds = new ChassisSpeeds(0,0,0);
    drive.runVelocity(speeds);  
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}