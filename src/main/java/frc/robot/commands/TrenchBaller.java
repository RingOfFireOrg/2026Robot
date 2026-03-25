package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Intake.Intake;

public class TrenchBaller {

  public static Command create(Turret turret, Indexer indexer, Transfer transfer, Intake intake) {

    double topRPM = -3000;
    double bottomRPM = 3000.0;

    return Commands.sequence(
        Commands.runOnce(() -> intake.setDeployPositionDeg(83), intake),

        Commands.runOnce(() -> turret.setShooterRPM(topRPM, bottomRPM), turret),

        Commands.waitSeconds(1.5),
 
        Commands.parallel(
            intake.rollersOut(),
            indexer.runPercent(0.8),
            transfer.runPercent(-0.8)
        ).withTimeout(4.0),

        Commands.runOnce(() -> turret.setShooterRPM(0, 0), turret)
    );
  }
}