package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake.Intake;

public class Rollin {

  public static Command create(Intake intake) {

    return intake.rollersOut();}}