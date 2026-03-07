// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndRange extends Command {
  /** Creates a new AimAndRange. */
  double rangingDistance = 0.5;//change ts
  double kp = 0.05;
  Drive drive;
  String camera;
  public AimAndRange(Drive drive, String camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.camera = camera;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!LimelightHelpers.getTV(camera)) return;
    double offBy = LimelightHelpers.getTX(camera);
    double offByDistance = LimelightHelpers.getTY(camera) + rangingDistance;
    drive.runVelocity(new ChassisSpeeds(kp * offByDistance, 0, kp*offBy));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
