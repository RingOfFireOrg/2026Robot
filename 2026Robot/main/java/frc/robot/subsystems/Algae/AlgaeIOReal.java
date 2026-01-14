package frc.robot.subsystems.Algae;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.LoggedTunableNumber;

public class AlgaeIOReal implements AlgaeIO {
    
    private final SparkMax algaePivotMotor;
    private final SparkMax leftAlgaeIntakeMotor;
    private final SparkMax rightAlgaeIntakeMotor;

    private final AbsoluteEncoder absEncoder;
    private SparkMaxConfig config = new SparkMaxConfig();
    private SparkMaxConfig intakeConfig = new SparkMaxConfig();

    public static final int algaePivotMotorCanID = 20;
    public static final int leftAlgaeIntakeMotorCanID = 21;
    public static final int rightAlgaeIntakeMotorCanID = 22;


    private LoggedTunableNumber kP = new LoggedTunableNumber("Algae/kP", 2.4);
    private LoggedTunableNumber kI = new LoggedTunableNumber("Algae/kI", 0.0);
    private LoggedTunableNumber kD = new LoggedTunableNumber("Algae/kD", 0.02);

    public double zeroOffset = 0.0;

    private double MAX_VELOCITY = 0.5; // in degrees per second
    private double MAX_ACCELERATION = MAX_VELOCITY*2; // degrees per second squared
    
    @SuppressWarnings("unused")
    private boolean atGoal = false;
    double targetAngle = 0.35;

    private final ProfiledPIDController profiledPidController = new ProfiledPIDController(
        1.4, 0.0, 0.00, // PID gains (adjust as needed)
        new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );
    ArmFeedforward feedforward = new ArmFeedforward(0, .16, 0, 0);

    double output = 0;
    SparkClosedLoopController closedLoopController;


    @SuppressWarnings("removal")
    public AlgaeIOReal() {
        algaePivotMotor = new SparkMax(algaePivotMotorCanID, MotorType.kBrushless);
        leftAlgaeIntakeMotor = new SparkMax(leftAlgaeIntakeMotorCanID, MotorType.kBrushless);
        rightAlgaeIntakeMotor = new SparkMax(rightAlgaeIntakeMotorCanID, MotorType.kBrushless);

        absEncoder = algaePivotMotor.getAbsoluteEncoder();
        absEncoder.getPosition();
 

        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
        .absoluteEncoder
            .inverted(false)
            .zeroOffset(0.7)
            // .positionConversionFactor(4.0 * Math.PI)
            // .velocityConversionFactor(7.0 * Math.PI ) // i aint counting the number of teeth on the gears
            //.zeroOffset(((absEncoder.getPosition() + 0.6 ) % 1 + 1) % 1)
        
            ; 
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kP.get(), kI.get(), kD.get())
            //.outputRange(-12, 12);   
            .outputRange(-1, 1)
            
            
            ;   
        ;
        intakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(17)
        ;

        


        algaePivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftAlgaeIntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightAlgaeIntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        
        zeroOffset = resetOffset();

        profiledPidController.disableContinuousInput();
        profiledPidController.setTolerance(0.05);

        closedLoopController = algaePivotMotor.getClosedLoopController();
        


    
        
    }

    @SuppressWarnings("removal")
    @Override
    public void updateInputs(AlgaeIOInputs inputs) {

        LoggedTunableNumber.ifChanged(
            hashCode(), 
            () -> {
                config.closedLoop.pid(kP.get(), kI.get(), kD.get());
                algaePivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }, 
            kP, kI, kD
        );

        output = profiledPidController.calculate(getAbsOffset(), targetAngle);
        atGoal = profiledPidController.atGoal();
        inputs.atGoal = profiledPidController.atGoal();
        Logger.recordOutput("Algae/AlgaePivot Value", algaePivotMotor.getAbsoluteEncoder().getPosition());
        Logger.recordOutput("Algae/Abs Encoder Raw", absEncoder.getPosition());
        Logger.recordOutput("Algae/Abs Encoder Offset FF", getAbsFFOffset());
        Logger.recordOutput("Algae/profiled output", output);


        
        if (DriverStation.isDisabled()) {
            algaePivotMotor.setVoltage(0);
            leftAlgaeIntakeMotor.setVoltage(0);
            rightAlgaeIntakeMotor.setVoltage(0);
        }



    }



    @Override
    public void setVoltage(double volts) {
        algaePivotMotor.setVoltage(volts/1.5 + feedforward.calculate(absEncoder.getPosition(), 0));
    }
    
    @Override
    public void setVoltageIntake(double volts) {
        leftAlgaeIntakeMotor.setVoltage(volts/1.2);
        rightAlgaeIntakeMotor.setVoltage(-volts/1.2);
    }

    @Override
    public void setVoltageLaunch(double left, double right) {
        leftAlgaeIntakeMotor.setVoltage(left);
        rightAlgaeIntakeMotor.setVoltage(right);
    }

    @Override
    public void setReference(double position) {
        // Removed conversion settings, use raw encoder values
        if (Math.abs(position - absEncoder.getPosition()) > 0.01) {

            closedLoopController.setSetpoint(position, ControlType.kPosition); 
            
        }
        else {
            algaePivotMotor.setVoltage(0+feedforward.calculate(absEncoder.getPosition(), 0));
        }
    }

    /* Set offset to value on startup */
    public double resetOffset() {
        return absEncoder.getPosition();
    }
    /* Get Absolute Encoder Offset Upon Startup wrapped around 0 to 1 */
    public double getAbsOffset() {
        return  ((absEncoder.getPosition()- zeroOffset + (0.35)) % 1 + 1) % 1; 
    }
    /* Get Absolute Encoder Offset for Feedforward Upon Startup wrapped around 0 to 1 
     * 0 should be parrallel to floor */
    public double getAbsFFOffset() {
        return  ((absEncoder.getPosition() - .6) % 1 + 1) % 1; 
    }

}