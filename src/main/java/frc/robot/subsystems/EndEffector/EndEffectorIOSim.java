package frc.robot.subsystems.EndEffector;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import java.util.function.Supplier;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EndEffectorIOSim implements EndEffectorIO {
  private DCMotorSim intakeMotor = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1.0),
    DCMotor.getNEO(1));
  @SuppressWarnings("unused")
  private final MutAngle m_angle = Degrees.mutable(0);


    //figure how to do ts normally bruh
  Angle zeroedAngle = Degrees.of(0);
  Angle stowedAngle = Degrees.of(-85); //pitch
  private Supplier<Double> elevatorHeight;
  private double appliedVolts = 0.0;
  @SuppressWarnings("unused")
  private final IntakeSimulation intakeSimulation;


  public EndEffectorIOSim(Supplier<Double> elevatorHeight,AbstractDriveTrainSimulation driveTrain) {
    this.elevatorHeight = elevatorHeight;
    this.intakeSimulation = IntakeSimulation.InTheFrameIntake(
      // Specify the type of game pieces that the intake can collect
      "Coral",
      // Specify the drivetrain to which this intake is attached
      driveTrain,
      // Specify width of the intake
      Meters.of(0.7),
      // The intake is mounted on the back side of the chassis
      IntakeSimulation.IntakeSide.FRONT,
      // The intake can hold up to 1 coral
      1);
  }
    


  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    //inputs.elevatorPositionMeters = intakeMotor.getPositionMeters();
    intakeMotor.setInputVoltage(appliedVolts);
    intakeMotor.update(0.02);

    inputs.angularPosition = intakeMotor.getAngularPosition().magnitude();
    // elevatorSimMotor.setInputVoltage(appliedVolts);
    // elevatorSimMotor.update(0.02);
    // inputs.angularPositionRot = elevatorSimMotor.getAngularPositionRotations();
    

    // inputs.positionRad = elevatorSimMotor.getAngularPositionRad();
    // inputs.velocityRadPerSec = elevatorSimMotor.getAngularVelocityRadPerSec();new Rotation3d(0.0, Math.sin(Timer.getTimestamp()) - 1.0, 0.0))
    inputs.appliedVolts = appliedVolts;
    inputs.endEffectorPose3d = new Pose3d(0.2,0,elevatorHeight.get() + 0.5, new Rotation3d(zeroedAngle, intakeMotor.getAngularPosition() , zeroedAngle));
    //inputs.endEffectorPose3d = new Pose3d(0.09,0,elevatorHeight.get(), new Rotation3d(0.0, Math.sin(Timer.getTimestamp()) - 1.0, 0.0));

    // inputs.currentAmps = elevatorSimMotor.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  public void setAngle(double angle) {
    intakeMotor.setAngle(Units.degreesToRadians(angle));
  }

  public Angle getAngleConvert(double angle) {
    Angle zeroedAngle = Degrees.of(angle);
    return zeroedAngle;
  }
}