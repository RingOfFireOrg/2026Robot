// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.LimelightFrontName;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShoot extends Command {
  private Turret turret;
  private Indexer indexer;

  private final double limelightMountAngle = 10;//change ts
  private final double limelightLensHeight = 21.5;//change ts
  private final double hubTagHeight = 44.5;//change ts

  public AutoShoot(Indexer indexer, Turret turret) {
    this.turret = turret;
    this.indexer = indexer;
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double verticalOffset = LimelightHelpers.getTY(LimelightFrontName);
    double angleToGoal = Units.degreesToRadians(limelightMountAngle+verticalOffset);
    double distance = (hubTagHeight - limelightLensHeight) / Math.tan(angleToGoal);
    double shooterRPM = (distance*1)+1;//change ts
    turret.runShooterRPM(()->shooterRPM, ()->shooterRPM);
    
    if((turret.getShooterBottomMeasuredRpm() - shooterRPM <= 20) || (turret.getShooterTopMeasuredRpm() - shooterRPM <= 20)) {
      indexer.setVolts(10);//change ts
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
