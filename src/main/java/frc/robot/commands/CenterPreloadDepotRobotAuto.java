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
    return Commands.sequence(
        new RobotDriveForTime(drive, 0.5, 0.0, 0.0, 3.25),//go back the first time
        Ballin.create(turret, indexer, transfer),
        new RobotDriveForTime(drive, 0.0, -0.5, 0.0, 3.0),//line up with depot
        Commands.runOnce(() -> intake.setDeployPositionDeg(83), intake),

        Commands.parallel(
          intake.rollersOut(),
          new RobotDriveForTime(drive, 0.25, 0.0, 0.0, 3.0)),//go into depot

        new RobotDriveForTime(drive, -0.25, 0, 0, 3.0),//away from depot
        new RobotDriveForTime(drive, 0, 0.5, 0, 3),//in front of ladder

        Ballin.create(turret, indexer, transfer)
    );
  }
}