package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.drive.Drive;

public class CenterPreloadRobotAuto {
  public static Command create(Drive drive, Turret turret, Indexer indexer, Transfer transfer) {
    return Commands.sequence(
        Ballin.create(turret, indexer, transfer),
        new RobotDriveForTime(drive, 1.0, 0.0, 0.0, 3)
    );
  }
}