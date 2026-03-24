// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndRange extends Command {
  /** Creates a new AimAndRange. */
  double rangingDistance = 6;//change ts
  double kpAim = 0.05;
  double kpRange = 0.025;
  boolean end = false;
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
    double offset = 0;
    double offBy = 0;
    int tagId = (int)LimelightHelpers.getFiducialID(camera);
    if(tagId == 26 || tagId == 10 || tagId == 5 || tagId == 2 || tagId == 21 || tagId == 18) offset = 0;
    else if(tagId == 9 || tagId == 25 || tagId == 11 || tagId == 27) offset = 2;
    else if (tagId == 8 || tagId == 24) offset = -2;
    else return;
    offBy = (((LimelightHelpers.getTX(camera)+offset))/29) * drive.getMaxAngularSpeedRadPerSec() * kpAim;
    
    
    drive.runVelocity(new ChassisSpeeds(0, 0, offBy));
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
