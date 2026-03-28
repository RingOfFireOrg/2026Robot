package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.Intake.Intake;

public class CenterPreloadRobotAuto {
  public static Command create(Drive drive, Turret turret, Indexer indexer, Transfer transfer, Intake intake) {
    return Commands.sequence(
        Commands.runOnce(() -> intake.setDeployPositionDeg(13), intake),
        new RobotDriveForTime(drive, 0.5, 0.0, 0.0, 2.8),
        Ballin.create(turret, indexer, transfer, intake)
    );
  }
}