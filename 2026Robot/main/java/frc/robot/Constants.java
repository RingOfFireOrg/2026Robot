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
    public static enum EndEffectorPositionState {
        STOWED,
        INTAKE,
        SCORE,
        SCOREL4
    }
    public static enum EndEffectorIntakeState {
        INTAKE,
        EJECT,
        HOLD,
        NONE
    }

    public static class OIConstants {
        public static final double controllerDeadband = 0.1;
    }

    public static class ElevatorHeights {
        public static final double STOWED = 0;
        public static final double INTAKE_HEIGHT = 0.583;
        public static final double L2 = 1.2;

        public static final double L3 = 3.5;
        public static final double L4 = 0;
        public static final double LOWER_ALGAE = 3.35;
        public static final double GROUND_ALGAE = 0.3;
        public static final double SCORE_ALGAE = 1.1;
        public static final double UPPER_ALGAE = 3.5;




    }
    public static class PivotAngles {
        public static final double STOWED = 0.42;
        public static final double INTAKE = 0.47;
        public static final double L2 = 0.575;
        public static final double L3 = 0.575;
        public static final double ALGAE = 0.39;
        public static final double GROUND_ALGAE = 0.535;




        public static final double Intake_Coral = 0.7;
        public static final double Eject_Coral = -0.7;
        public static final double Maintain_Coral = 0.3;

    }

    public static class AlgaeAngles {
        public static final double STOWED = 0.88;
        public static final double LOWER_ALGAE = 0.693;
        public static final double UPPER_ALGAE = 0.777; 
        public static final double GROUND_ALGAE = 0.595; 
        public static final double SCORE_ALGAE = 0.693; 


    }

    public static final double X_REEF_ALIGNMENT_P = 2;
	public static final double Y_REEF_ALIGNMENT_P = 1.7;
	public static final double ROT_REEF_ALIGNMENT_P = 0.02;

	public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
	public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1.2;
	public static final double X_SETPOINT_REEF_ALIGNMENT = 0.14;  // Vertical pose
	public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
	public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.275;  // Horizontal pose
	public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.04;

	public static final double DONT_SEE_TAG_WAIT_TIME = 1;
	public static final double POSE_VALIDATION_TIME = 0.3;
}
