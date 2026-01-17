// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  final String cam = "limelight-tag@2";
  public Robot() {
    LimelightHelpers.setPipelineIndex(cam, 0);
  }


  @Override
  public void robotPeriodic() {
    
    boolean tv = LimelightHelpers.getTV(cam);
    int id = (int)LimelightHelpers.getFiducialID(cam);
    double tx = LimelightHelpers.getTX(cam);
    double ty = LimelightHelpers.getTY(cam);
    double ta = LimelightHelpers.getTA(cam);

    System.out.print("Hublock target: " + tv + " Target ID: " + id + "\n");
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
