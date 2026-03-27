package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.commands.TurretLock;
import frc.robot.subsystems.drive.Drive;
@SuppressWarnings("unused")

public class BumpDepotAuto {
    public static Command create(Drive drive, Turret turret, Indexer indexer, Transfer transfer, Intake intake) {
    return Commands.sequence(
        Commands.waitSeconds(5),

        new RobotDriveForTime(drive, 0.5, 0.0, 0.0, 3),//back up and shoot preload
        new TurretLock(turret),
        Ballin.create(turret, indexer, transfer),

        Commands.parallel(//go into depot and shoot
            new RobotDriveForTime(drive, 0.5, 0.0, 0.0, 3),
            Commands.runOnce(() -> intake.setDeployPositionDeg(83), intake),
            intake.rollersOut(),
            new TurretLock(turret)),
            
        Ballin.create(turret, indexer, transfer)        
        );
    }
    
}
