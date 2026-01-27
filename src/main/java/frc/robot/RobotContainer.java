package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import java.security.Principal;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeAngles;
import frc.robot.Constants.ElevatorHeights;
import frc.robot.Constants.PivotAngles;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.AlignToReef.reefSide;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIOReal;

import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorIO;
import frc.robot.subsystems.EndEffector.EndEffectorIOReal;
import frc.robot.subsystems.EndEffector.EndEffectorIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.commands.HubLock;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.commands.AlignToHub;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Transfer.Transfer;
import frc.robot.subsystems.LED.LED;
import frc.robot.commands.SetLED;
import frc.robot.subsystems.Intake.Intake;



@SuppressWarnings("unused")
public class RobotContainer {
    private final Drive drive;
    private Vision vision;
    private Climber climber;
    private SwerveDriveSimulation driveSimulation = null;
    private Turret turret;
    private Command hubLock;
    private Indexer indexer;
    private Transfer transfer;
    private LED led;
    private Command SetLED;
    private Intake intake;


    // Controller
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandJoystick climberController = new CommandJoystick(2);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    private final double maxSpeed = 0.6;
    private final double standardSpeed = 0.7;
    private final double turnSpeed = 0.5;

    public RobotContainer() {

        switch (Constants.currentMode) {
            case REAL:
                drive = new Drive(
                    new GyroIONavX(),
                    new ModuleIOSpark(0),
                    new ModuleIOSpark(1),
                    new ModuleIOSpark(2),
                    new ModuleIOSpark(3),
                    (pose) -> {
                });
                climber = new Climber(new ClimberIOReal());
                turret = new Turret();
                indexer = new Indexer();
                transfer = new Transfer();
                led = new LED();
                intake = new Intake();
                
                this.vision = new Vision(
                    drive,
                    new VisionIOLimelight("limelight-tag", drive::getRotation),
                    new VisionIOPhotonVision(camera0Name, robotToCamera0),
                    new VisionIOPhotonVision(camera1Name, robotToCamera1)
                );
               
                hubLock = new HubLock(turret, this.vision, 0);
                turret.setDefaultCommand(hubLock);
                SetLED = new SetLED(led, 0, 0, 0, false);

                break;
            case SIM:
                this.driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig,
                    new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                this.drive = new Drive(
                    new GyroIOSim(driveSimulation.getGyroSimulation()),
                    new ModuleIOSim(driveSimulation.getModules()[0]),
                    new ModuleIOSim(driveSimulation.getModules()[1]),
                    new ModuleIOSim(driveSimulation.getModules()[2]),
                    new ModuleIOSim(driveSimulation.getModules()[3]),
                    driveSimulation::setSimulationWorldPose);
                turret = new Turret();
                indexer = new Indexer();
                transfer = new Transfer();
                intake = new Intake();


                this.vision = new Vision(
                    drive,
                    new VisionIOPhotonVisionSim(
                        camera0Name, robotToCamera0,
                        driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(
                        camera1Name, robotToCamera1,
                        driveSimulation::getSimulatedDriveTrainPose)
                );
                hubLock = new HubLock(turret, this.vision, 0);
                turret.setDefaultCommand(hubLock);

                break;
            default:
                // Replayed robot, disable IO implementations
                this.drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    (pose) -> {});
                //this.vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                this.vision = new Vision(
                    drive,
                    new VisionIOPhotonVisionSim(
                        camera0Name, robotToCamera0,
                        driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(
                        camera1Name, robotToCamera1,
                        driveSimulation::getSimulatedDriveTrainPose)
);

                break;
        }


        setNamedCommands();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        autoChooser.addDefaultOption("do nothing", Commands.none());
        
        //autoChooser.addOption("MECK) Align Right",

//        new PathPlannerAuto("range into reef", false) // Get in Vision Range of the reef & prep L2
 //           .andThen( new AlignToReef(drive, reefSide.RIGHT).withTimeout(3)) // Align using LL
  //          .andThen( new PathPlannerAuto("range & station", false)) //Resetting Odo, run up on the reef and drop, and then back out and go to feeder station
   //         .andThen( new PathPlannerAuto("range & score", false)) //Resetting Odo, run up on the reef and drop, and then back out and go to feeder station
    //    );

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(drive)
        );
        autoChooser.addOption("Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization(drive)
        );
        autoChooser.addOption("Mid Climb", 
            Commands.sequence(
                hubLock.withTimeout(1.0),
                turret.runShooterPercent(0.85).withTimeout(1.5),
                Commands.parallel(
                    indexer.runPercent(0.6),
                    transfer.runPercent(0.6)
                ).withTimeout(0.9),
                Commands.waitSeconds(0.2),
                new PathPlannerAuto("Mid Climb")
                //climber.runPercent(0.4).withTimeout(2.0)
                //)
        ));
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        double standardSpeed = 0.8;
        double turnSpeed = 0.5;

        drive.setDefaultCommand(DriveCommands.joystickDrive(
         drive,
            () -> {
                double maxSpeedX = (1 - driver.getLeftTriggerAxis()) * (standardSpeed + (1-standardSpeed) * driver.getRightTriggerAxis());
                return MathUtil.applyDeadband(MathUtil.clamp(-driver.getLeftY(), -maxSpeedX, maxSpeedX), 0.1);
            },
            () -> {
                double maxSpeedY = (1 - driver.getLeftTriggerAxis()) * (standardSpeed + (1-standardSpeed) * driver.getRightTriggerAxis());
                return MathUtil.applyDeadband(MathUtil.clamp(-driver.getLeftX(), -maxSpeedY, maxSpeedY), 0.1);
            },
            () -> {
                double maxSpeedTheta = (1 - driver.getLeftTriggerAxis()) * (turnSpeed + (1-turnSpeed) * driver.getRightTriggerAxis());
                return MathUtil.applyDeadband(MathUtil.clamp(-driver.getRightX(), -maxSpeedTheta, maxSpeedTheta), 0.1);
            }
        ));


        driver.povRight().whileTrue(DriveCommands.joystickDriveRobotOriented(drive, () -> 0, () -> -0.5 + -(driver.getRightTriggerAxis()/2), () -> 0));
        driver.povLeft().whileTrue(DriveCommands.joystickDriveRobotOriented(drive, () -> 0, () -> 0.5 + (driver.getRightTriggerAxis()/2), () -> 0));
        driver.povUp().whileTrue(DriveCommands.joystickDriveRobotOriented(drive, () -> 0.5 + (driver.getRightTriggerAxis()/2), () -> 0, () -> 0));
        driver.povDown().whileTrue(DriveCommands.joystickDriveRobotOriented(drive, () -> -0.5 + -(driver.getRightTriggerAxis()/2), () -> 0, () -> 0));


        
        driver.rightBumper().whileTrue(DriveCommands.joystickDriveAtAngle(drive, 
        () -> MathUtil.applyDeadband(MathUtil.clamp(-driver.getLeftY(),-maxSpeed,maxSpeed), 0.1),
        () -> MathUtil.applyDeadband(MathUtil.clamp(-driver.getLeftX(),-maxSpeed,maxSpeed), 0.1),
        () -> {
            boolean isFlipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
            return new Rotation2d(Math.toRadians(isFlipped ? -125 + 180 : -125));
        }));

        driver.leftBumper().whileTrue(DriveCommands.joystickDriveAtAngle(drive, 
        () -> MathUtil.applyDeadband(MathUtil.clamp(-driver.getLeftY(),-maxSpeed,maxSpeed), 0.1),
        () -> MathUtil.applyDeadband(MathUtil.clamp(-driver.getLeftX(),-maxSpeed,maxSpeed), 0.1),
        () -> {
            boolean isFlipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
            return new Rotation2d(Math.toRadians(isFlipped ? 125 + 180 : 125));
        }));


        //Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.resetOdometry(
                driveSimulation
                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
                : () -> drive.resetOdometry(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        
        driver.back().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));


        if (Constants.currentMode == Constants.Mode.REAL) {

            /* Driver - Align to the Hub */
            driver.x()
            .whileTrue(new AlignToHub(drive, vision, 0));
            //.onTrue(new AlignToReef(drive, reefSide.LEFT).withTimeout(1.2));
            
            /* Operator - Turret Manual Control */
            //if (turret != null) {
            //    operator.leftBumper().whileTrue(
            //        Commands.run(() -> turret.setDutyCycle(0.2), turret)
            //    )
            //    .onFalse(
            //        Commands.runOnce(turret::stop, turret)
            //    );
            //}

            /* Operator – Turret Hub Lock */
            //if (turret != null) {
            //    operator.rightBumper().whileTrue(hubLock);
            //    operator.rightTrigger().whileTrue(turret.runShooterPercent((0.8)));
            //}


            operator.y().whileTrue(transfer.runPercent(0.6)); //Transfer
            operator.x().whileTrue(Commands.parallel(transfer.runPercent(0.6), intake.rollersIn(0.6))); //Transfer and Intake
            operator.b().whileTrue(Commands.parallel(transfer.runPercent(-0.6), indexer.runPercent(-0.6))); //Transfer and indexer out
            operator.a().whileTrue(Commands.parallel(transfer.runPercent(-0.6), intake.rollersOut(0.6))); //transfer and outtake 

            operator.povUp().onTrue(hubLock); // reapplys hublock if switched off
            operator.povRight().whileTrue(intake.deployIn(0.6));//intake comes in
            operator.povLeft().whileTrue(intake.deployOut(0.6));//intake goes out
            operator.povDown().whileTrue(Commands.run(() -> {}, turret));//offs hublock

            //operator.leftStick().whileTrue(climber.runTeleop(() -> -MathUtil.applyDeadband(operator.getLeftY(), 0.12)));//climber
            operator.leftStick().whileTrue(Commands.parallel(
                climber.runTeleop(() -> -MathUtil.applyDeadband(operator.getLeftY(), 0.12)),
                Commands.runOnce(() -> led.setSolid(255,0,0)).repeatedly()))
            .onFalse(Commands.runOnce(led::restoreAlliance));

            operator.rightBumper().whileTrue(Commands.runEnd(() -> turret.setDutyCycle(+0.25), turret::stop, turret));//manual turret turning
            operator.leftBumper().whileTrue(Commands.runEnd(() -> turret.setDutyCycle(-0.25), turret::stop, turret));
            //operator.rightTrigger().whileTrue(turret.runShooterPercent(0.9));
            operator.rightTrigger().whileTrue(Commands.parallel(turret.runShooterPercent(0.9), 
            Commands.waitSeconds(1.0).andThen(Commands.runOnce(() -> led.setSolid(0, 255, 0)).repeatedly())))//shooter
            .onFalse(Commands.runOnce(led::restoreAlliance));
            operator.leftTrigger().whileTrue(Commands.parallel(transfer.runPercent(0.6), indexer.runPercent(0.6)));//transfer and indexer up


        


            /* Climbing Controls */
            climberController.axisMagnitudeGreaterThan(Joystick.AxisType.kY.value, 0.2)
            .and(climberController.button(1))
            .and(climberController.povCenter())
            .whileTrue(climber.runTeleop(() -> -climberController.getY()))
            .onFalse(climber.runTeleop(() -> 0));
            /*
             * EXAMPLE FROM 2025 ^
             */

        }         
        else if (Constants.currentMode == Constants.Mode.SIM) {

            // PathConstraints constraints = new PathConstraints(3.0, 4.0,
            //     Units.degreesToRadians(540), Units.degreesToRadians(720)
            // );
            // try { controller.b().whileTrue(
            //     AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("toReed"),constraints));
            // } 
            // catch (FileVersionException | IOException | ParseException e) {e.printStackTrace();}

            // driver.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.1)
            // .whileTrue(claw.moveElevatorUpCommand())
            // .onFalse(claw.holdElevatorPositionCommand());

            
            
            
            // driver.leftTrigger().whileTrue(getAutonomousCommand());
    
        }

    }

    public void setNamedCommands() {
/* 
        NamedCommands.registerCommand(
        "AimAndShoot",
        hubLock.withTimeout(1.0)
            .andThen(turret.runShooterPercent(0.8).withTimeout(2.0))
            .andThen(Commands.parallel(
                indexer.runPercent(0.6),
                transfer.runPercent(0.6)
            ).withTimeout(1.0))
        );
*/
        //NamedCommands.registerCommand("Align Center", new AlignToReef(drive, reefSide.CENTER).withTimeout(2));
        //NamedCommands.registerCommand("Align Left", new AlignToReef(drive, reefSide.LEFT).withTimeout(2));
        //NamedCommands.registerCommand("Align Right", new AlignToReef(drive, reefSide.RIGHT).withTimeout(2));


    }
    
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }























    /* -------------------------------------------------------------------------------------------------- */
    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;
        drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }


    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
            "FieldSimulation/Coral",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
            "FieldSimulation/Algae",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }}





