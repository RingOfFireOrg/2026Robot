package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    // private DCMotorSim elevatorSimMotor =
    // new DCMotorSim(
    //   LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1.0),
    //   DCMotor.getNEO(1));

  private ElevatorSim elevatorSim = 
    new ElevatorSim( LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1.0), DCMotor.getNEO(1), 0, 0.56, false, 0);
  private PIDController controller = new PIDController(0.05, 0, 0.001);
  
    

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
   
    //controller.setSetpoint(appliedVolts);
    inputs.elevatorPositionMeters = elevatorSim.getPositionMeters();
    elevatorSim.setInputVoltage(appliedVolts);
    elevatorSim.update(0.02);
    // elevatorSimMotor.setInputVoltage(appliedVolts);
    // elevatorSimMotor.update(0.02);
    // inputs.angularPositionRot = elevatorSimMotor.getAngularPositionRotations();
    

    // inputs.positionRad = elevatorSimMotor.getAngularPositionRad();
    // inputs.velocityRadPerSec = elevatorSimMotor.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.elevatorPose3d = new Pose3d(0,0,elevatorSim.getPositionMeters(),new Rotation3d());
    inputs.controllerSetpoint = controller.getSetpoint();
    inputs.controllerOutput = controller.calculate(elevatorSim.getPositionMeters());
    // inputs.currentAmps = elevatorSimMotor.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public double getHeight(double height) {
    height = elevatorSim.getPositionMeters();
    return height;
  }

  @Override
  public void setHeight(double height) {
    appliedVolts = controller.calculate(elevatorSim.getPositionMeters(), height);
  }
}