// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.drive.Drive;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberAuto extends SequentialCommandGroup {
  public static Command create(Drive drive, Climber climber) {
    return Commands.sequence(
        new RobotDriveForTime(drive, 0.0, 0.25, 0.0, 11.7).alongWith(climber.goTop()),//back up and shoot preload
        new WaitCommand(1),
        climber.goBottom()    
        );
    }
}
