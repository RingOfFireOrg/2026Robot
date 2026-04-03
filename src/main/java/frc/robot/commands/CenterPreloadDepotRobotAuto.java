package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.Intake.Intake;

public class CenterPreloadDepotRobotAuto {
  public static Command create(Drive drive, Turret turret, Indexer indexer, Transfer transfer, Intake intake) {
    return Commands.parallel(
      new TurretLock(turret), // turret should lock on the whole auto
      Commands.sequence(
        Commands.runOnce(() -> intake.setDeployPositionDeg(13), intake),

        new RobotDriveForTime(drive, 1, 0.0, 0.0, 1.625),//go back the first time
        Ballin.create(turret, indexer, transfer,intake),
        new RobotDriveForTime(drive, 0.0, -1, 0.0, 2.25),//line up with depot

        Commands.runOnce(() -> intake.setDeployPositionDeg(83), intake),
        

        Commands.parallel(
          intake.rollersOut(),
          new RobotDriveForTime(drive, 0.25, 0.0, 0.0, 3.75)).withTimeout(3.8),//go into depot

        new RobotDriveForTime(drive, -0.5, 0, 0, 1.875),//away from depot
        new RobotDriveForTime(drive, 0, 1, 0, 2.25),//in front of ladder

        Commands.runOnce(() -> intake.setDeployPositionDeg(13), intake),

        Ballin.create(turret, indexer, transfer,intake)
      ));
  }
}