package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.Turret.Turret;

public class TrenchBaller {

  public static Command create(Turret turret, Indexer indexer, Transfer transfer) {

    double topRPM = 3250;
    double bottomRPM = 2250.0;

    return Commands.sequence(

        Commands.runOnce(() -> turret.setShooterRPM(topRPM, bottomRPM), turret),

        Commands.waitSeconds(1.5),

        Commands.parallel(
            indexer.runPercent(0.8),
            transfer.runPercent(0.8)
        ).withTimeout(4.0),

        Commands.runOnce(() -> {
          turret.setShooterRPM(0, 0);
        }, turret)
    );
  }
}