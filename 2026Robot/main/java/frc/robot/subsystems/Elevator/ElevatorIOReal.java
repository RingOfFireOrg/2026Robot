package frc.robot.subsystems.Elevator;


import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ElevatorFeedforward;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ElevatorIOReal implements ElevatorIO {

    /*
    *  
     Here for refrence, elevator motor has been switched to talonfx
    *  
    */
    private final SparkMax elevatorMotor;
    private final RelativeEncoder encoder;
    private SparkMaxConfig config = new SparkMaxConfig();
    private SparkClosedLoopController closedLoopController;
    ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);

    MutVoltage appliedVoltage = Volts.mutable(0);
    MutAngle angle = Radians.mutable(0);
    MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

    private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(10, 20));
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    //public static final int elevatorCanIDleft = 14;
    public static final int elevatorCanID/*right*/ = 15;

    @SuppressWarnings("removal")
    public ElevatorIOReal() {
        kP = 0.5; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000015; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        elevatorMotor = new SparkMax(elevatorCanID, MotorType.kBrushless);
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            ;

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(kP)
            .i(kI)
            .d(kD)
            .velocityFF(kFF)
            .outputRange(-0.8, 1);

        config.encoder.positionConversionFactor(1);
        //config.encoder.velocityConversionFactor(1);

        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        encoder = elevatorMotor.getEncoder();
        closedLoopController = elevatorMotor.getClosedLoopController();

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

        // routine = new SysIdRoutine(
        //     new SysIdRoutine.Config(),
        //     new SysIdRoutine.Mechanism(elevatorMotor::setVoltage, 
        //     log -> {log.motor("ElevatorMotor").voltage(appliedVoltage.mut_replace(
        //         elevatorMotor.get() * RobotController.getBatteryVoltage(), Volts))
        //     .angularPosition(angle.mut_replace(encoder.getPosition(), Rotations))
        //     .angularVelocity(velocity.mut_replace(encoder.getVelocity(), RotationsPerSecond));
        // }, this 
        //     ));
    
    }

    @SuppressWarnings("removal")
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

        Logger.recordOutput("ElevatorPosition", elevatorMotor.getEncoder().getPosition());
        Logger.recordOutput("referenceposi", setpoint.position);
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { 
            config.closedLoop.p(p); 
            kP = p; 
            elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        }
        if((i != kI)) { 
            config.closedLoop.i(i); 
            kI = i; 
            elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        }
        if((d != kD)) { 
            config.closedLoop.d(d); 
            kD = d; 
            elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        }
        
        // }
        if((ff != kFF)) { config.closedLoop.velocityFF(ff); kFF = ff; 
            elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        }




    }

    @Override
    public void setVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
    }

    @Override
    public double getHeight(double height) {
        return height;
    }



    public void sysIDTest() {

    }

    @Override
    public void setHeight(double userGoal) {
        goal = new TrapezoidProfile.State(userGoal, 0);
        setpoint = new TrapezoidProfile.State(encoder.getPosition(), 0);
        setpoint = profile.calculate(1, setpoint, goal);

        closedLoopController.setSetpoint(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0);

    }

    public void zeroElevator() {
        encoder.setPosition(0);
    }
}