package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.Turret.Turret;

public class BallinTest {

  public static Command create(Turret turret, Indexer indexer, Transfer transfer) {
    return Commands.sequence(
        shootAtRPM(turret, indexer, transfer, 3000, 2000, "Shot1"),
        Commands.waitSeconds(1.0),

        shootAtRPM(turret, indexer, transfer, 3250, 2250, "Shot2"),
        Commands.waitSeconds(1.0),

        shootAtRPM(turret, indexer, transfer, 3500, 2500, "Shot3"),
        Commands.waitSeconds(1.0),

        shootAtRPM(turret, indexer, transfer, 3750, 2750, "Shot4"),
        Commands.waitSeconds(1.0),

        Commands.runOnce(() -> {
          turret.stopShooter();
          System.out.println("[BallinTest] Done testing all RPMs");
        }, turret)
    );
  }

  private static Command shootAtRPM(
      Turret turret,
      Indexer indexer,
      Transfer transfer,
      double topRPM,
      double bottomRPM,
      String label) {

    return Commands.sequence(
        Commands.runOnce(() -> {
          System.out.println("[BallinTest] " + label + " starting");
          System.out.println("[BallinTest] topRPM=" + topRPM + " bottomRPM=" + bottomRPM);
          Logger.recordOutput("BallinTest/" + label + "/TopSetpoint", topRPM);
          Logger.recordOutput("BallinTest/" + label + "/BottomSetpoint", bottomRPM);
          Logger.recordOutput("BallinTest/CurrentShotLabel", label);
          turret.setShooterRPM(topRPM, bottomRPM);
        }, turret),

        Commands.waitSeconds(1.5),

        Commands.runOnce(() -> {
          double topMeas = turret.getShooterTopMeasuredRpm();
          double bottomMeas = turret.getShooterBottomMeasuredRpm();

          System.out.println(
              "[BallinTest] " + label
                  + " pre-feed measured top=" + topMeas
                  + " bottom=" + bottomMeas
                  + " time=" + Timer.getFPGATimestamp());

          Logger.recordOutput("BallinTest/" + label + "/TopMeasuredBeforeFeed", topMeas);
          Logger.recordOutput("BallinTest/" + label + "/BottomMeasuredBeforeFeed", bottomMeas);
        }),

        Commands.parallel(
            indexer.runPercent(0.8),
            transfer.runPercent(0.8)
        ).withTimeout(1.25),

        Commands.runOnce(() -> {
          turret.stopShooter();
          System.out.println("[BallinTest] " + label + " finished");
        }, turret),

        Commands.waitSeconds(0.5)
    );
  }
}