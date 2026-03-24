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
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double offset = 0;
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
    if(LimelightHelpers.getFiducialID(LimelightFrontName) == 26) {
      double output = turretPID.calculate(
          LimelightHelpers.getTX(LimelightFrontName));
      System.out.println("Turret PID output: " + output);
      if (output > 1) output = 1;
      if (output < -1) output = -1;
      turret.setTurret(
        output*0.07);
    
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
