package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private static final int LED_PWM_PORT = 1;
  private static final int LED_LENGTH = 60;

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  private Alliance lastAlliance = null;

  public LED() {
    led = new AddressableLED(LED_PWM_PORT);
    buffer = new AddressableLEDBuffer(LED_LENGTH);

    led.setLength(buffer.getLength());
    setSolid(255, 255, 255);
    led.start();
  }

  @Override
  public void periodic() {
    Alliance alliance = DriverStation.getAlliance().orElse(null);

    if (alliance != lastAlliance) {
      if (alliance == Alliance.Red) {
        setSolid(255, 0, 0);
      } else if (alliance == Alliance.Blue) {
        setSolid(0, 0, 255);
      } else {
        setSolid(255, 255, 255);
      }
      lastAlliance = alliance;
    }
  }

  public void setSolid(int r, int g, int b) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, r, g, b);
    }
    led.setData(buffer);
  }

  public void restoreAlliance() {
    lastAlliance = null;
  }
}
