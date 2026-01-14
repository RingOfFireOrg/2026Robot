package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

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
import frc.robot.subsystems.Algae.Algae;
import frc.robot.subsystems.Algae.AlgaeIOReal;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIOReal;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIORealTalon;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
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

@SuppressWarnings("unused")
public class RobotContainer {
    private final Drive drive;
    private Vision vision;
    private Elevator elevator;
    private Climber climber;
    private SwerveDriveSimulation driveSimulation = null;
    private EndEffector EndEffector;
    private Algae algae;

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
                elevator = new Elevator(new ElevatorIORealTalon());
                EndEffector = new EndEffector(new EndEffectorIOReal());
                climber = new Climber(new ClimberIOReal());
                algae = new Algae(new AlgaeIOReal());
                // this.vision = new Vision(
                //     drive,
                //     new VisionIOLimelight("limelight-tag", drive::getRotation)
                //    // ,new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation)
                // );
               

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

                this.vision = new Vision(
                    drive,
                    new VisionIOPhotonVisionSim(
                        camera0Name, robotToCamera0,
                        driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(
                        camera1Name, robotToCamera1,
                        driveSimulation::getSimulatedDriveTrainPose));
                this.elevator = new Elevator(new ElevatorIOSim() {});
                this.EndEffector = new EndEffector(new EndEffectorIOSim(() -> elevator.getHeight(), driveSimulation) {
                     });

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
                this.vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                this.elevator = new Elevator(new ElevatorIO(){});
                this.EndEffector = new EndEffector(new EndEffectorIO() {});
            break;
        }


        setNamedCommands();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        
        autoChooser.addOption("MECK) Align Right",
        new PathPlannerAuto("range into reef", false) // Get in Vision Range of the reef & prep L2
            .andThen( new AlignToReef(drive, reefSide.RIGHT).withTimeout(3)) // Align using LL
            .andThen( new PathPlannerAuto("range & station", false)) //Resetting Odo, run up on the reef and drop, and then back out and go to feeder station
            .andThen( new PathPlannerAuto("range & score", false)) //Resetting Odo, run up on the reef and drop, and then back out and go to feeder station
        );

        autoChooser.addOption("MECK) Align Left",
        new PathPlannerAuto("range into reef", true) // Get in Vision Range of the reef & prep L2
            .andThen( new AlignToReef(drive, reefSide.RIGHT).withTimeout(3)) // Align using LL
            .andThen( new PathPlannerAuto("range & station", true)) //Resetting Odo, run up on the reef and drop, and then back out and go to feeder station
            .andThen( new PathPlannerAuto("range & score", true)) //Resetting Odo, run up on the reef and drop, and then back out and go to feeder station
        );

        autoChooser.addOption("MECK) LeftSide Auto",
        new PathPlannerAuto("MECK) RightSide Auto", true) // Get in Vision Range of the reef & prep L2
        );

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(drive)
        );
        autoChooser.addOption("Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization(drive)
        );

        // autoChooser.addOption(
        //     "Drive SysId (Quasistatic Forward)",
        //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        // );
        // autoChooser.addOption(
        //     "Drive SysId (Quasistatic Reverse)",
        //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        // );
        // autoChooser.addOption("Drive SysId (Dynamic Forward)",
        //     drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        // );
        // autoChooser.addOption("Drive SysId (Dynamic Reverse)",
        //     drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        // );

        // autoChooser.addOption("rightSideAuto",
        //     new PathPlannerAuto("Auto Name", true);
        // );

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

            driver.x()
            .onTrue(new AlignToReef(drive, reefSide.LEFT).withTimeout(1.2));
            driver.y()
            .onTrue(new AlignToReef(drive, reefSide.CENTER).withTimeout(1.2));
            driver.b()
            .onTrue(new AlignToReef(drive, reefSide.RIGHT).withTimeout(1.2));

            operator.y()
            .whileTrue(algae.runTeleop(() -> 0.4))
            .onFalse(algae.runTeleop(() -> 0.0));
            operator.a()
            .whileTrue(algae.runTeleop(() -> -0.4))
            .onFalse(algae.runTeleop(() -> 0.0));

            operator.x()
            .whileTrue(algae.runTeleopIntake(() -> -1))
            .onFalse(algae.runTeleopIntake(() -> 0.0));
            operator.b()
            .whileTrue(algae.runTeleopIntake(() -> 1))
            .onFalse(algae.runTeleopIntake(() -> 0.0));

            climberController.button(6)
            .whileTrue(algae.runTeleopIntake(() -> -1))
            .onFalse(algae.runTeleopIntake(() -> 0.0));

            climberController.button(5)
            .whileTrue(algae.runTeleopIntake(() -> 1))
            .onFalse(algae.runTeleopIntake(() -> 0.0));

            operator.povUp()
            .whileTrue(elevator.setHeight(ElevatorHeights.L3))
            .whileTrue(algae.runPosition(() -> AlgaeAngles.STOWED))
            .onTrue(EndEffector.angle(PivotAngles.L3))
            ;
    
            operator.povLeft()
            .whileTrue(elevator.setHeight(ElevatorHeights.L2))
            .whileTrue(algae.runPosition(() -> AlgaeAngles.STOWED))
            .onTrue(EndEffector.angle(PivotAngles.L2))
            ;

            operator.povRight()
            .whileTrue(elevator.setHeight(ElevatorHeights.LOWER_ALGAE))
            .whileTrue(algae.runPositionandIntake(() -> AlgaeAngles.LOWER_ALGAE, () -> .9))
            .whileTrue(algae.runPosition(() -> AlgaeAngles.LOWER_ALGAE)) // Not supposed to be called twice, but it works so I will leave it alone
            .onTrue(EndEffector.angle(PivotAngles.STOWED))
            ;

            operator.povDown()
            .whileTrue(elevator.setHeight(0))
            .whileTrue(algae.runPosition(() -> AlgaeAngles.STOWED))
            .onTrue(EndEffector.angle(PivotAngles.STOWED))
           ;

            /* Intake Coral */
            operator.leftBumper()
            .onTrue(EndEffector.ejecter(0.7))
            .onFalse(EndEffector.ejecter(0));

            /* Intake Coral */
            operator.rightBumper()
            .onTrue(EndEffector.ejecter(-0.7))
            .onFalse(EndEffector.ejecter(0));

            /* Intaking Setup Button - Move elevator to intake height, move intake to intake angle, and Intake coral */
            operator.axisMagnitudeGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.1)
            .whileTrue(elevator.setHeight(ElevatorHeights.INTAKE_HEIGHT))
            .onTrue(algae.runPosition(() -> AlgaeAngles.STOWED))
            .onTrue(EndEffector.setAngleandIntake(PivotAngles.INTAKE, 0.7))
            .onFalse(EndEffector.ejecter(0))
            ;

            /* Manual Control for Elevator */
            operator.axisMagnitudeGreaterThan(XboxController.Axis.kLeftY.value, 0.1)
            .whileTrue(elevator.runTeleop(() -> -operator.getLeftY()/1.4))
            .onFalse(elevator.runTeleop(() -> 0))
            ;

            /* Manual Control for Pivot intake */
            operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, 0.1)
            .whileTrue(EndEffector.runTeleop(() -> -operator.getRightY()/3, ()-> 0, () -> 0))
            .onFalse(EndEffector.runTeleop(() -> 0, ()-> 0, () -> 0));

            
            /* Eject Coral CMD */
            operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightTrigger.value, 0.1)
            .whileTrue(EndEffector.runTeleop(() -> 0, ()-> 0, () -> -operator.getRightTriggerAxis()))
            .onFalse(EndEffector.runTeleop(() -> 0, ()-> 0, () -> 0));

            /* Climbing Controls */
            climberController.axisMagnitudeGreaterThan(Joystick.AxisType.kY.value, 0.2)
            .and(climberController.button(1))
            .and(climberController.povCenter())
            .whileTrue(climber.runTeleop(() -> -climberController.getY()))
            .onFalse(climber.runTeleop(() -> 0));

            climberController.button(11).onTrue(climber.runLauncher(-0.3)).onFalse(climber.runLauncher(0.0));
            climberController.button(12).onTrue(climber.runLauncher(0.3)).onFalse(climber.runLauncher(0.0));


            /*
             * Elevator: Move to Lower Algae Level
             * EndEffector: Angle to Stow
             * Algae: Move to Lower Algae Level, Spin Wheels in
             */
            climberController.povUp()
            .whileTrue(elevator.setHeight(ElevatorHeights.UPPER_ALGAE))
            .whileTrue(algae.runPositionandIntake(() -> AlgaeAngles.UPPER_ALGAE, () -> .9))
            .whileTrue(algae.runPosition(() -> AlgaeAngles.UPPER_ALGAE))
            .onTrue(EndEffector.angle(PivotAngles.STOWED))
            ;
            /*
             * Elevator: Move to Ground Algae Level
             * EndEffector: Angle to Stow
             * Algae: Move to Ground Algae Level, Spin Wheels in
             */
            climberController.povDown()
            .whileTrue(elevator.setHeight(ElevatorHeights.GROUND_ALGAE))
            .whileTrue(algae.runPositionandIntake(() -> AlgaeAngles.GROUND_ALGAE, () -> .75))
            .whileTrue(algae.runPosition(() -> AlgaeAngles.GROUND_ALGAE))
            .onTrue(EndEffector.angle(PivotAngles.GROUND_ALGAE))
            ;


            /*
             * Elevator: Move to Lower Algae Level
             * EndEffector: Angle to Stow
             * Algae: Move to Lower Algae Level, Spin Wheels in
             */
            climberController.povLeft()
            .whileTrue(elevator.setHeight(ElevatorHeights.LOWER_ALGAE))
            .whileTrue(algae.runPositionandIntake(() -> AlgaeAngles.LOWER_ALGAE, () -> .75))
            .whileTrue(algae.runPosition(() -> AlgaeAngles.LOWER_ALGAE))
            .onTrue(EndEffector.angle(PivotAngles.STOWED))
            ;


            /* Robot to Scoring
             * Elevator: Move to Lower Algae Level
             * EndEffector: Angle to Stow
             * Algae: Move to Lower Algae Level, Spin Wheels in
             */            
            climberController.povRight()
            .whileTrue(elevator.setHeight(ElevatorHeights.SCORE_ALGAE))
            .whileTrue(algae.runPositionandIntake(() -> AlgaeAngles.SCORE_ALGAE, () -> .75))
            .whileTrue(algae.runPosition(() -> AlgaeAngles.SCORE_ALGAE))
            .onTrue(EndEffector.angle(PivotAngles.STOWED))
            ;

            climberController.button(10).whileTrue(climber.autoLaunch());

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
         * Raise Elevator to L3
         * Pivot Endeffector to L3
         */
        NamedCommands.registerCommand("Prep_L3", 
        elevator.runOnceHeight(ElevatorHeights.L3)
        .alongWith(EndEffector.angle(0.52))
        //.alongWith(algae.runPosition(() -> AlgaeAngles.STOWED))
        .alongWith(Commands.print("Prep_L3"))
        );
        
        /*
         * Raise Elevator to L2d
         * Pivot Endeffector to L2
         */
        NamedCommands.registerCommand("Prep_L2", 
        elevator.runOnceHeight(ElevatorHeights.L2)
        .alongWith(EndEffector.angle(PivotAngles.L2))
        //.alongWith(algae.runPosition(() -> AlgaeAngles.STOWED))
        .alongWith(Commands.print("NamedCommand: Prep_L2"))
        );

        /*
         * Raise Elevator to Intake Height
         * Pivot Endeffector to L3
         * Start Intaking
         */
        NamedCommands.registerCommand("Intake", 
        elevator.runOnceHeight(ElevatorHeights.INTAKE_HEIGHT)
        .alongWith(EndEffector.angle(PivotAngles.INTAKE))
        //-.alongWith(algae.runPosition(() -> AlgaeAngles.STOWED))
        .andThen(EndEffector.ejecter(0.7))
        .alongWith(Commands.print("NamedCommand: Intake"))
        );

        /*
         * Raise Elevator to ALgae Lower
         * Pivot Endeffector to Stowed
         */
        NamedCommands.registerCommand("Prep_Algae", 
        elevator.runOnceHeight(ElevatorHeights.LOWER_ALGAE)
        .alongWith(EndEffector.angle(PivotAngles.STOWED))
        .alongWith(algae.runPositionandIntake(() -> AlgaeAngles.LOWER_ALGAE, () -> .9))
        .alongWith(Commands.print("NamedCommand: Lower Algae"))
        .withTimeout(2)
        );
        

        NamedCommands.registerCommand("Reset",
        elevator.runOnceHeight(ElevatorHeights.STOWED)
        .andThen(EndEffector.angle(PivotAngles.STOWED))
        //.alongWith(algae.runPosition(() -> AlgaeAngles.STOWED))
        .andThen(EndEffector.ejecter(PivotAngles.Maintain_Coral))
        .alongWith(Commands.print("NamedCommand: Reset"))
        );  

        NamedCommands.registerCommand("Ejecter_Eject",
        EndEffector.ejecter(-0.9)
        .alongWith(Commands.print("NamedCommand: Ejecter_Eject"))
        ); 

        NamedCommands.registerCommand("Ejecter_EjectMAX",
        EndEffector.ejecter(-1)
        .alongWith(Commands.print("NamedCommand: Ejecter_EjectMAX"))
        ); 


        NamedCommands.registerCommand("Ejecter_Stop",
        EndEffector.ejecter(0)
        .alongWith(Commands.print("NamedCommand: Ejecter_Stop"))
        ); 

        NamedCommands.registerCommand("Align Center", new AlignToReef(drive, reefSide.CENTER).withTimeout(2));
        NamedCommands.registerCommand("Align Left", new AlignToReef(drive, reefSide.LEFT).withTimeout(2));
        NamedCommands.registerCommand("Align Right", new AlignToReef(drive, reefSide.RIGHT).withTimeout(2));


    }
    
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public EndEffector getEffector() {
        return EndEffector;
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
    }




}
