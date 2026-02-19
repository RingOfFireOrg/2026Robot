// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
@SuppressWarnings("unused")
public class AlignToHub extends Command {
  private PIDController xController,yController,rotController;
  private Timer dontSeeTagTimer, stopTimer;
  private Drive drive;
  private double blueTagID = 9;
  private double redTagID = 25;
  private LoggedTunableNumber xP = new LoggedTunableNumber("Align/xP", Constants.HUB_ALIGNMENT_P);
  private LoggedTunableNumber yP = new LoggedTunableNumber("Align/yP", Constants.Y_HUB_ALIGNMENT_P);
  private LoggedTunableNumber zP = new LoggedTunableNumber("Align/zP", Constants.ROT_HUB_ALIGNMENT_P);
  public AlignToHub(Drive drive) {
    xController = new PIDController(xP.getAsDouble(), 0, 0);
    yController = new PIDController(yP.getAsDouble(), 0, 0);
    rotController = new PIDController(zP.getAsDouble(), 0, 0);
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    xController = new PIDController(xP.getAsDouble(), 0, 0);
    yController = new PIDController(yP.getAsDouble(), 0, 0);
    rotController = new PIDController(zP.getAsDouble(), 0, 0);

    rotController.setSetpoint(0);
    rotController.setTolerance(1.2);

    xController.setSetpoint(0.14);
    xController.setTolerance(0.02);
    yController.setSetpoint(0);

    yController.setTolerance(0.04);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
