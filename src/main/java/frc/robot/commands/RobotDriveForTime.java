package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class RobotDriveForTime extends Command {
  private final Drive drive;
  private final double vx;
  private final double vy;
  private final double omega;
  private final double seconds;
  private final Timer timer = new Timer();

  public RobotDriveForTime(Drive drive, double vx, double vy, double omega, double seconds) {
    this.drive = drive;
    this.vx = vx;
    this.vy = vy;
    this.omega = omega;
    this.seconds = seconds;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= seconds;
  }
}