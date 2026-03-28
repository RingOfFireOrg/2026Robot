package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Intake.Intake;

public class Ballin {

  public static Command create(Turret turret, Indexer indexer, Transfer transfer, Intake intake) {

    double topRPM = -2775;
    double bottomRPM = 2775;
    return Commands.sequence(

        Commands.runOnce(() -> turret.setShooterRPM(topRPM, bottomRPM), turret),

        Commands.waitSeconds(0.5),

        Commands.parallel(
            intake.rollersOut(),
            indexer.runPercent(0.9),
            transfer.runPercent(-0.7)
        ).withTimeout(5.0),

        Commands.runOnce(() -> {
          turret.setShooterRPM(0, 0);
        }, turret)
    );
  }
}