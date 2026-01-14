package frc.robot.subsystems.EndEffector;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.Constants.EndEffectorIntakeState;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class EndEffectorIOReal implements EndEffectorIO {

    private final SparkMax EndEffectorMotor;
    private final SparkMax EjectMotor;
    private SparkMaxConfig config = new SparkMaxConfig();
    double ffOutput;
    private static final int EndEffectorCanID = 17;
    private static final int EjectCanID = 18;

    private final DutyCycleEncoder absEncoder;
    public double zeroOffset = 0.0;
    private int AbsEncoderDIOID = 1;
    public double targetAngle = 0.0; // Target position in degrees
    double ppidOutput = 0;

    private double MAX_VELOCITY = 0.3; // in degrees per second
    private double MAX_ACCELERATION = 1; // degrees per second squared

    private LoggedTunableNumber kP = new LoggedTunableNumber("Coral/kP", 2.9);
    private LoggedTunableNumber kI = new LoggedTunableNumber("Coral/kI", 0.1);
    private LoggedTunableNumber kD = new LoggedTunableNumber("Coral/kD", 0.01);

    private final ProfiledPIDController profiledPidController = new ProfiledPIDController(
        1.5, 0.0, 0.02, // PID gains (adjust as needed)
        new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );
    ArmFeedforward feedforward = new ArmFeedforward
    (0, .18, 0, 0);
    double output = 0;


    private PIDController pidController; 
    private boolean enableHoming = false; // In case operator Takes manuel control

    

    @SuppressWarnings("removal")
    public EndEffectorIOReal() {

        EndEffectorMotor = new SparkMax(EndEffectorCanID, MotorType.kBrushless);
        EjectMotor = new SparkMax(EjectCanID, MotorType.kBrushless);

        absEncoder = new DutyCycleEncoder(AbsEncoderDIOID);
        

        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(17);


        config.encoder.positionConversionFactor(1);
        //config.encoder.velocityConversionFactor(1);
        pidController = new PIDController(kP.get(), kI.get(), kD.get());
        EndEffectorMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        EjectMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        absEncoder.setInverted(true);
        

        pidController.disableContinuousInput();
        pidController.setTolerance(0.03);

        profiledPidController.disableContinuousInput();
        profiledPidController.setTolerance(0.005);

        profiledPidController.setIZone(0.03);
        profiledPidController.setIntegratorRange(-1.6, 1.6);
        
        zeroOffset = resetOffset();
        targetAngle = 0.35;
        pidController.setIZone(0.3);
        pidController.setIntegratorRange(-1.6, 1.6);

    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        //double output = pidController.calculate(getPositionDegrees(), targetAngle);
        
        ffOutput = feedforward.calculate(
            Math.toRadians(getAbsFFOffset()*360),0.0
        );

        LoggedTunableNumber.ifChanged(
            hashCode(), 
            () -> {
                pidController = new PIDController(kP.get(), kI.get(), kD.get());
            }, 
            kP, kI, kD
        );

        if (zeroOffset == 0.0 || zeroOffset == 0.35) {
            /* Dosen't always properly apply the offset, so run in periodic if thats the case */
            zeroOffset = resetOffset();
        }
        if (getAbsOffset() < 0.265) {
            zeroOffset = resetOffset();

            enableHoming = false;
            EndEffectorMotor.set(0);
            System.out.println("ENCODER PASSED USABLE AREA");
        }
        else if (enableHoming && absEncoder.isConnected()) {
            output = pidController.calculate(getAbsOffset(), targetAngle);
            ppidOutput = profiledPidController.calculate(getAbsOffset(), targetAngle);

            EndEffectorMotor.set(-MathUtil.clamp(MathUtil.applyDeadband(output, 0.06) + (ffOutput/3.7), -0.4,0.4));
            Logger.recordOutput("test_Target output", output);
        }
        else if (enableHoming && !absEncoder.isConnected()) {
            System.out.println("ENCODER IS NOT CONNECTED, HOMING DISABLED");
            enableHoming = false;
            EndEffectorMotor.set(0);
        
        }

        if (DriverStation.isDisabled()) {
            enableHoming = false;
            EndEffectorMotor.set(0);
            EjectMotor.set(0);
        }
        
        
        Logger.recordOutput("en_ Encoder Value With Offset", getAbsOffset());
        Logger.recordOutput("en_ Encoder Value Raw", absEncoder.get());
        Logger.recordOutput("en_ Encoder Connected", absEncoder.isConnected());
        Logger.recordOutput("test_1 Targeting Angle:", pidController.getSetpoint());
        Logger.recordOutput("test_2 Supposed to target:", targetAngle);
        Logger.recordOutput("test_3 Encoder Raw Value", absEncoder.get());
        Logger.recordOutput("test_4 Encoder Offset Value", getAbsOffset());
        Logger.recordOutput("test_5 Error", pidController.getError());
        Logger.recordOutput("test_4 Encoder Offset", zeroOffset);
        Logger.recordOutput("test_5 Encoder Offset With .35", zeroOffset + .35);
        Logger.recordOutput("test_ppid output", ppidOutput);
        Logger.recordOutput("test_ff output", getAbsFFOffset());
        Logger.recordOutput("test_ff Angle", ffOutput);
        Logger.recordOutput("test_ppid + ff output", ppidOutput + ffOutput);
        Logger.recordOutput("test_x Homing Enabled", enableHoming);
        Logger.recordOutput("test_X At goal", pidController.atSetpoint());



    }

    @Override
    public void setVoltage(double volts) {
        enableHoming = false;
        EndEffectorMotor.setVoltage(volts/3 + ffOutput);
    }

    @Override
    public void setVoltageEject(double volts, double ejectsVolts) {
        enableHoming = false;
        EndEffectorMotor.setVoltage(volts/3 + ffOutput);
        EjectMotor.setVoltage(ejectsVolts/4);
    }

    @Override
    public void setAngle(double angle) {
        targetAngle = angle;
        enableHoming = true;
    }
    
    @Override
    public void setAngleandIntake(double angle, double volts) {
        setAngle(angle);
        ejecter(volts*12);
    }

    @Override
    public void ejecter(double volts) {
        EjectMotor.setVoltage(volts/3);
        System.out.println(EjectMotor.getAppliedOutput());

    }

    public void endEffectorState(double volts, EndEffectorIntakeState state) {
        switch (state) {
            case EJECT:
                break;
            case INTAKE:
                break;
            case NONE:
                break;
            default:
                System.out.println("Unknown state: " + state);
        }

    }
         

    public double resetOffset() {
        return absEncoder.get() ;
    }

    @Override
    public void resetEncoder() {
        zeroOffset = resetOffset();

    }

    @Override
    public void enableHoming(boolean enable){
        enableHoming = enable;
    }

    public double getPositionDegrees() {
        return getAbsOffset() * 360;
    }

    public double getAbsOffset() {
        return  ((absEncoder.get()- zeroOffset +0.35) % 1 + 1) % 1; 
    }

    
    public double getAbsFFOffset() {
        return  ((absEncoder.get()- zeroOffset - .2) % 1 + 1) % 1; 
    }


}