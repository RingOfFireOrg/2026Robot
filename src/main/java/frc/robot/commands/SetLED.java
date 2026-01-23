package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED.LED;

public class SetLED extends Command {
  private final LED led;
  private final int r;
  private final int g;
  private final int b;
  private final boolean restoreOnEnd;

  public SetLED(LED led, int r, int g, int b, boolean restoreOnEnd) {
    this.led = led;
    this.r = r;
    this.g = g;
    this.b = b;
    this.restoreOnEnd = restoreOnEnd;
    addRequirements(led);
  }

  @Override
  public void initialize() {
    led.setSolid(r, g, b);
  }

  @Override
  public void end(boolean interrupted) {
    if (restoreOnEnd) {
      led.restoreAlliance();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
