package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;

public class DriveAim {

  private static final PIDController rotPid = new PIDController(0.03, 0.0, 0.002);

  static {
    rotPid.setTolerance(0.5);
  }

  public static Command create(
      Drive drive,
      String limelightName,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {

    return Commands.runOnce(() -> {
      rotPid.reset();
    }).andThen(Commands.run(() -> {

      double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
      double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.1);

      boolean hasTarget = LimelightHelpers.getTV(limelightName);
      double tx = LimelightHelpers.getTX(limelightName);

      double omega = 0.0;
      if (hasTarget) {
        omega = rotPid.calculate(tx, 0.0);
        omega = MathUtil.clamp(omega, -6.0, 6.0);
      }

      ChassisSpeeds speeds = new ChassisSpeeds(
          x * drive.getMaxLinearSpeedMetersPerSec(),
          y * drive.getMaxLinearSpeedMetersPerSec(),
          omega
      );

      boolean isFlipped = DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red;

      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds,
          isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()
      );

      drive.runVelocity(speeds);

    }, drive));
  }
}