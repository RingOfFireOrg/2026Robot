package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


import frc.robot.subsystems.Intake.Intake;

public class Rollin {

  public static Command create(Intake intake) {
    return Commands.sequence(
    Commands.runOnce(() -> intake.deployOut()),
    intake.rollersOut().withTimeout(3));
}}