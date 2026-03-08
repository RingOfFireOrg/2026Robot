package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.Turret.Turret;

public class WakePreload {

  public static Command create(Drive drive, Turret turret, Indexer indexer, Transfer transfer) {
    return Commands.sequence(
        new DriveBack(drive, -20.0, 1.0)
        //new Turn180(drive),
        //Ballin.create(turret, indexer, transfer)
    );
  }

  private static class DriveBack extends Command {
    private final Drive drive;
    private final double distance;
    private final double speed;
    private Pose2d startPose;

    public DriveBack(Drive drive, double distance, double speed) {
      this.drive = drive;
      this.distance = distance;
      this.speed = speed;
      addRequirements(drive);
    }

    @Override
    public void initialize() {
      startPose = drive.getPose();
    }

    @Override
    public void execute() {
      drive.runVelocity(new ChassisSpeeds(-speed, 0, 0));
    }

    @Override
    public void end(boolean interrupted) {
      drive.runVelocity(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
      return drive.getPose().getTranslation().getDistance(startPose.getTranslation()) >= distance;
    }
  }

  private static class Turn180 extends Command {
    private final Drive drive;
    private final PIDController pid = new PIDController(3.0, 0.0, 0.1);
    private double target;

    public Turn180(Drive drive) {
      this.drive = drive;
      pid.enableContinuousInput(-Math.PI, Math.PI);
      pid.setTolerance(Math.toRadians(2));
      addRequirements(drive);
    }

    @Override
    public void initialize() {
      target = drive.getRotation().getRadians() + Math.PI;
      target = MathUtil.angleModulus(target);
      pid.reset();
    }

    @Override
    public void execute() {
      double omega = pid.calculate(drive.getRotation().getRadians(), target);
      omega = MathUtil.clamp(omega, -2.5, 2.5);
      drive.runVelocity(new ChassisSpeeds(0, 0, omega));
    }

    @Override
    public void end(boolean interrupted) {
      drive.runVelocity(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
      return pid.atSetpoint();
    }
  }
}