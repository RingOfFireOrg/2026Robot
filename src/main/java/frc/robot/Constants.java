package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;


public final class Constants {
    public static final boolean tuningMode = true;

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    /*
     * 20 algae pivor
     * 21 left algae spin
     * 22 right algae spin
     */

    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }
    public static class OIConstants {
        public static final double controllerDeadband = 0.1;
    }
}
