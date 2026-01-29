package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.DigitalOutput;

public class LedModeBus {
  private final DigitalOutput b0;
  private final DigitalOutput b1;
  private final DigitalOutput b2;
  private final DigitalOutput b3;

  public LedModeBus(int dio0, int dio1, int dio2, int dio3) {
    b0 = new DigitalOutput(dio0);
    b1 = new DigitalOutput(dio1);
    b2 = new DigitalOutput(dio2);
    b3 = new DigitalOutput(dio3);
  }

  public void setMode(int mode) {
    int m = mode & 0xF;
    b0.set((m & 0x1) != 0);
    b1.set((m & 0x2) != 0);
    b2.set((m & 0x4) != 0);
    b3.set((m & 0x8) != 0);
  }
}
