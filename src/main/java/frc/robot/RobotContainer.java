package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.LimelightFrontName;
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
import com.pathplanner.lib.path.PathPlannerPath;

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
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Climber.Climber;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
//import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
//import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.LimelightHelpers;
import frc.robot.commands.HubLock;
import frc.robot.commands.HubTurn;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
//import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.commands.AimAndRange;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.AlignWhileMoving;
import frc.robot.commands.AutoShoot;
//import frc.robot.commands.AutoShoot;
//import frc.robot.commands.PhotonAlign;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Transfer.Transfer;
//import frc.robot.subsystems.LED.LED;
//import frc.robot.commands.SetLED;
import frc.robot.subsystems.Intake.Intake;
//import frc.robot.subsystems.LED.LedManager;
//import frc.robot.subsystems.LED.LedModeBus;
import frc.robot.commands.HubTurn;
import frc.robot.commands.Ballin;
import frc.robot.commands.Rollin;
import frc.robot.commands.Shuttle;
import frc.robot.commands.BallinTest;
import frc.robot.commands.DriveAim;
import frc.robot.commands.WakePreload;
import frc.robot.commands.TrenchBaller;
import frc.robot.commands.TurretFindAndLock;
import frc.robot.commands.TurretLock;
import frc.robot.commands.RobotDriveForTime;
import frc.robot.commands.CenterPreloadRobotAuto;
import frc.robot.commands.CenterPreloadDepotRobotAuto;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;





@SuppressWarnings("unused")
public class RobotContainer {
    private final Drive drive;
    private Vision vision;
    private Climber climber;
    private SwerveDriveSimulation driveSimulation = null;
    private Turret turret;
    //private Command hubLock;
    private Indexer indexer;
    private Transfer transfer;
    //private LED led;
    //private Command SetLED;
    private Intake intake;
    //private LedManager dioLed;
    private double lastValidDistanceM = 2.0;
    private double lastSeenTimeSec = 0.0;
    private static final double kTargetHoldTimeSec = 0.25;
    private double lastPrintTimeSec = 0.0;
    private Command HubTurn;




    // Controller
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandXboxController climberController = new CommandXboxController(2);
    
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    private final double maxSpeed = 0.6;
    //private final double standardSpeed = 0.7;
    //private final double turnSpeed = 0.5;

    public RobotContainer() {

        switch (Constants.currentMode) {
            case REAL:
                drive = new Drive(
                    //new GyroIONavX(),
                    new GyroIOPigeon2(DriveConstants.pigeonCanId, DriveConstants.canivoreBusName),
                    //new GyroIOPigeon2(0, "FRC-3459-PT-CANivore"),
                    new ModuleIOSpark(0),
                    new ModuleIOSpark(1),
                    new ModuleIOSpark(2),
                    new ModuleIOSpark(3),
                    (pose) -> {
                });
                climber = new Climber();
                turret = new Turret();
                indexer = new Indexer();
                transfer = new Transfer();
                //led = new LED();
                //dioLed = new LedManager(new LedModeBus(0, 1, 2, 3));
                intake = new Intake();
                
                this.vision = new Vision(
                    drive,
                    new VisionIOLimelight("limelight-tag", drive::getRotation),
                    new VisionIOPhotonVision(camera1Name, robotToCamera1),
                    new VisionIOPhotonVision(camera0Name, robotToCamera0)
                );
               
                //hubLock = new HubLock(turret, this.vision, 0);
                //turret.setDefaultCommand(hubLock);

                //turret.setDefaultCommand(hubLock());
                //SetLED = new SetLED(led, 0, 0, 0, false);
                //HubTurn = new HubTurn(drive, "limelight-tag");


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
                climber = new Climber();


                //this.vision = new Vision(s
                  //  drive,
                    //new VisionIOPhotonVisionSim(
                      //  camera0Name, robotToCamera0,
                        //driveSimulation::getSimulatedDriveTrainPose),
                    //new VisionIOPhotonVisionSim(
                      //  camera1Name, robotToCamera1,
                       // driveSimulation::getSimulatedDriveTrainPose)
                //);
                //hubLock = new HubLock(turret, this.vision, 0);
                //turret.setDefaultCommand(hubLock);
                //turret.setDefaultCommand(hubLock());



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
                //this.vision = new Vision(
                  //  drive
                    //new VisionIOPhotonVisionSim(
                      //  camera0Name, robotToCamera0,
                        //driveSimulation::getSimulatedDriveTrainPose),
                    //new VisionIOPhotonVisionSim(
                      //  camera1Name, robotToCamera1,
                        //driveSimulation::getSimulatedDriveTrainPose)
                //);

                break;
        }


        setNamedCommands();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        autoChooser.addDefaultOption("do nothing", Commands.none());
        autoChooser.addOption("Wake Trench Preload", TrenchBaller.create(turret, indexer, transfer, intake));
        autoChooser.addOption("V1", new PathPlannerAuto("Ver1", false));
        autoChooser.addOption("V2", new PathPlannerAuto("Ver2", false));
        autoChooser.addOption("PreloadTrench", new PathPlannerAuto("WakePreloadDepotTrench"));
        autoChooser.addOption("Center Preload + Depot V1", new PathPlannerAuto("CenterDepotAutoV1", false)); 
        autoChooser.addOption("Center Preload + Depot V2", new PathPlannerAuto("CenterDepotAutoV2", false));     
        autoChooser.addOption("rolling", Rollin.create(intake));
        autoChooser.addOption("Trench Depot", new PathPlannerAuto("TrenchDepotAuto", false));
        autoChooser.addOption("DriveTester", new PathPlannerAuto("New Auto", false));
        autoChooser.addOption("tester", new PathPlannerAuto("testerauto", false));
        autoChooser.addOption("robotTest", new RobotDriveForTime(drive, 1.0, 0.0, 0.0, 3.0));
        autoChooser.addOption("Robot Center Preload", CenterPreloadRobotAuto.create(drive, turret, indexer, transfer));
        autoChooser.addOption("Robot Center Preload Depot", CenterPreloadDepotRobotAuto.create(drive, turret, indexer, transfer, intake));
        /* 
        autoChooser.addOption("Preload", new PathPlannerAuto("CenterPreloadAuto", false));
        autoChooser.addOption("Preload and Depot V2(use this one)", new PathPlannerAuto("CenterPreloadDepotAuto", false));
        autoChooser.addOption("TrenchTest", new PathPlannerAuto("WakeTrenchTest", false));
        autoChooser.addOption("BallinTest", new PathPlannerAuto("New Auto", false));
        autoChooser.addOption("Preload,Depot,Trench", Rollin.create(intake));
        autoChooser.addOption("test Preload", new PathPlannerAuto("PreloadTest", false));*/
        ////autoChooser.addOption("Limelight score(DONT USE)", new PathPlannerAuto().andThen());
        
        

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(drive)
        );
        autoChooser.addOption("Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization(drive)
        );

        //configureButtonBindings();
    
    
    
    
        /* 
        autoChooser.addOption("Mid Climb", 
            Commands.sequence(
                hubLock().withTimeout(1.0),
                turret.runShooterPercent(0.85).withTimeout(1.5),
                Commands.parallel(
                    indexer.runPercent(0.6),
                    transfer.runPercent(0.6)
                ).withTimeout(0.9),
                Commands.waitSeconds(2.0),
                //new PathPlannerAuto("Mid Climb")
                climber.runPercent(0.2).withTimeout(2.0)
                )
        );*/
        configureButtonBindings();
    }
 
    private void configureButtonBindings() {

        double standardSpeed = 0.8;
        double turnSpeed = 0.5;

        //turret.setDefaultCommand(new TurretLock(turret));

        drive.setDefaultCommand(DriveCommands.joystickDrive(
         drive,
            () -> {
                double maxSpeedX = (1 - driver.getLeftTriggerAxis()) * (standardSpeed + (1-standardSpeed) * driver.getRightTriggerAxis());
                return -1*MathUtil.applyDeadband(MathUtil.clamp(driver.getLeftY(), -maxSpeedX, maxSpeedX), 0.1);
            },
            () -> {
                double maxSpeedY = (1 - driver.getLeftTriggerAxis()) * (standardSpeed + (1-standardSpeed) * driver.getRightTriggerAxis());
                return -1*MathUtil.applyDeadband(MathUtil.clamp(driver.getLeftX(), -maxSpeedY, maxSpeedY), 0.1);
            },
            () -> {
                double maxSpeedTheta = (1 - driver.getLeftTriggerAxis()) * (turnSpeed + (1-turnSpeed) * driver.getRightTriggerAxis());
                return MathUtil.applyDeadband(MathUtil.clamp(-driver.getRightX(), -maxSpeedTheta, maxSpeedTheta), 0.1);
            }
        ));


        driver.povLeft().whileTrue(DriveCommands.joystickDriveRobotOriented(drive, () -> 0, () -> -0.5 + -(driver.getRightTriggerAxis()/2), () -> 0));
        driver.povRight().whileTrue(DriveCommands.joystickDriveRobotOriented(drive, () -> 0, () -> 0.5 + (driver.getRightTriggerAxis()/2), () -> 0));
        driver.povDown().whileTrue(DriveCommands.joystickDriveRobotOriented(drive, () -> 0.5 + (driver.getRightTriggerAxis()/2), () -> 0, () -> 0));
        driver.povUp().whileTrue(DriveCommands.joystickDriveRobotOriented(drive, () -> -0.5 + -(driver.getRightTriggerAxis()/2), () -> 0, () -> 0));


        
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
        //Takes over drive completely and ranges into hub
        driver.a().whileTrue(new AlignToHub(drive, vision, 0));
        driver.x().whileTrue(drive.stopX());
        //Only takes over rotation allows driver to continue to move while aligning
        driver.b().whileTrue(new AlignWhileMoving(drive, vision, 0, driver));


        //Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.resetOdometry(
                driveSimulation
                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
                : () -> drive.resetOdometry(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        
        //driver.back().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        driver.back().onTrue(
            Commands.runOnce(
                () -> drive.resetOdometry(
                    Constants.currentMode == Constants.Mode.SIM
                        ? driveSimulation.getSimulatedDriveTrainPose()
                        : new Pose2d(drive.getPose().getTranslation(), new Rotation2d())
            ),
            drive
        ).ignoringDisable(true)
    );


        if (Constants.currentMode == Constants.Mode.REAL) {

            /* Driver - Align to the Hub */
            //driver.x().whileTrue(new AlignToHub(drive, vision, 0));
            //driver.y().whileTrue(new PhotonAlign(drive, vision, 1));
            //driver.a().whileTrue(new PhotonAlign(drive, vision, 2));
            //driver.b().whileTrue(
                //DriveAim.create(drive, "limelight-tag", () ->  -driver.getLeftY(), () ->  -driver.getLeftX()).onlyIf(() -> LimelightHelpers.getTV("limelight-tag")));

   
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


            //operator.y().whileTrue(transfer.runPercent(0.6)); //Transfer
            operator.y().whileTrue(Commands.parallel(indexer.runVelocityRpm(indexer::getFeedRpm), transfer.runPercent(transfer::getFeedPercent))); //indexer and spindexer up
            operator.x().whileTrue(intake.rollersOut()); //Intake
            operator.a().whileTrue(new TurretLock(turret));
            //operator.a().whileTrue(Commands.parallel(transfer.runPercent(transfer::getFeedPercent), indexer.runPercent(indexer::getReversePercent))); //Transfer and indexer out
            //operator.b().whileTrue(intake.rollersIn()); //outtake 
            //operator.b().whileTrue(Commands.parallel(intake.shakeBalls(), intake.rollersOut()));


            //operator.povUp().onTrue(hubLock); // reapplys hublock if switched off
            //operator.povUp().onTrue(Commands.runOnce(() -> turret.setDefaultCommand(hubLock())));
            //operator.povUp().whileTrue
            operator.povUp().whileTrue(new TurretFindAndLock(turret));
                //Commands.parallel(indexer.runVelocityRpm(indexer::getFeedRpm),transfer.runPercent(0.8),intake.rollersIn()));
            //intake and transfer in, indexer up
            operator.povDown().whileTrue(Commands.parallel(indexer.runVelocityRpm(indexer::getReverseRpm),transfer.runPercent(-0.8),intake.rollersOut()));
            //intake and transfer out, indexer down
            operator.povRight().onTrue(intake.retractIn());//intake comes in
            operator.povLeft().onTrue(intake.deployOut());//intake goes out

            operator.rightStick().whileTrue(
                turret.runEnd(() -> {
                    turret.setDutyCycle(MathUtil.applyDeadband(operator.getRightX(), 0.05)*0.4);
                },
                    turret::stopTurret));     

            operator.leftStick().whileTrue(
                climber.runEnd(() -> {
                    climber.runPercent(MathUtil.applyDeadband(operator.getLeftY(), 0.05)*0.5);
                },
                    climber::stop));

            operator.rightBumper().whileTrue(turret.goToTurretAngle(90));
    

            operator.leftBumper().whileTrue(turret.goToTurretAngle(-90));
                //new Shuttle(turret,indexer, transfer));

             



            //operator.rightTrigger().whileTrue(turret.runShooterPercent(0.9));
            
            
            operator.rightTrigger().whileTrue(turret.runShooterRPM(turret::getDashboardTopRpm, turret::getDashboardBottomRpm));
            operator.back().onTrue(Commands.runOnce(() -> turret.zeroTurret(), turret));
            //run shooter at rpm determined by vision, run indexer when shooter is right speed
            operator.leftTrigger().whileTrue(new AutoShoot(indexer, turret, transfer));
            
         


           // Climbing Controls
            climberController.povRight().onTrue(intake.retractIn());//intake comes in
            climberController.povLeft().onTrue(intake.deployOut());//intake goes out
            climberController.povUp().onTrue(climber.goTop());//go to top
            climberController.povDown().onTrue(climber.goBottom());//go to bottom

            climberController.y().whileTrue(climber.runPercent(0.6));//climber up manual
            climberController.a().whileTrue(climber.runPercent(-0.6));//climber down manual

            climberController.back().whileTrue(turret.goToTurretAngle(0.0));
            climberController.x().whileTrue(turret.goToTurretAngle(-30.0));
            climberController.b().whileTrue(turret.goToTurretAngle(30.0));
            climberController.rightBumper().whileTrue(Commands.runOnce(() -> turret.zeroTurret(), turret));

            /*
             * EXAMPLE FROM 2025 ^
             */

        }         
        /*else if (Constants.currentMode == Constants.Mode.SIM) {

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
    
        }*/

    }

    public void setNamedCommands() {
        NamedCommands.registerCommand("Ballin", Ballin.create(turret, indexer, transfer));
        NamedCommands.registerCommand("Rollin", Rollin.create(intake));
        NamedCommands.registerCommand("BallinTest", BallinTest.create(turret, indexer, transfer));
        NamedCommands.registerCommand("Aim And Range", new AimAndRange(drive, LimelightFrontName));
        NamedCommands.registerCommand("Intake Out", intake.deployOut());
        NamedCommands.registerCommand("Intake In", intake.retractIn());
        NamedCommands.registerCommand("Turret Lock", new TurretLock(turret));

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

    public Drive getDrive() {
    return drive;
    } 

    private Command hubLock() {
    return new HubLock(turret, vision, 0);
}


    //public void ledPeriodic() {
      //  if (dioLed != null) {
        //    dioLed.periodic();
  //}
//}

























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





