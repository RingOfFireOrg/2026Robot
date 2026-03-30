// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.LimelightFrontName;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShoot extends Command {
  private Turret turret;

  private final double limelightMountAngle = 10;//change ts
  private final double limelightLensHeight = 21.5;
  private final double hubTagHeight = 44.5;

  public AutoShoot(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!LimelightHelpers.getTV(LimelightFrontName)) {
      turret.stopShooter();
      return;
    }
    double verticalOffset = LimelightHelpers.getTY(LimelightFrontName);
    double angleToGoal = Units.degreesToRadians(limelightMountAngle+verticalOffset);
    if (Math.abs(Math.tan(angleToGoal)) < 1e-6) {
      turret.stopShooter();
      return;
    }
    double distance = (hubTagHeight - limelightLensHeight) / Math.tan(angleToGoal);
    //double shooterRPM = (distance*92.5)+1990;//change ts
    double shooterRPM = (distance*turret.autoShoot.getDouble(1000))+turret.autoShoot.getDouble(1000);
    System.out.println("ty=" + verticalOffset + " distance=" + distance + " shooterRPM=" + shooterRPM);
    turret.setShooterRPM(-shooterRPM, shooterRPM);
    System.out.println(
    "top=" + turret.getShooterTopMeasuredRpm()
    + " bottom=" + turret.getShooterBottomMeasuredRpm()
    + " target=" + shooterRPM);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
