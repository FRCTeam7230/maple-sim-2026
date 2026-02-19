// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlignToBump;
import frc.robot.commands.AlignToBump2;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SpamShootCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AIRobotInSimulation;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.Optional;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;


import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.awt.Font;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

import javax.swing.JFrame;
import javax.swing.JLabel;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private SwerveDriveSimulation driveSimulation = null;
    private final Vision vision;

    // Controller
    private final Boolean controllerMode = true;
    private final GenericHID controller = new GenericHID(0);
   private  GenericHIDSim controllerSim = new GenericHIDSim(controller);
        
   // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
        JFrame frame = new JFrame();
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // AIRobotInSimulation a = new AIRobotInSimulation(

        //         , getAutonomousCommand(), null, getAutonomousCommand(), null, 0)
        Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
        arena.isActive(true);
        arena.setShouldRunClock(false);
        arena.setEfficiencyMode(true);
        SimulatedArena.overrideInstance(arena);
        

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3),
                        (pose) -> {});

                this.vision = new Vision(
                        drive,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                        new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

                break;

            case SIM:
                // create a maple-sim swerve drive simulation instance
                this.driveSimulation =
                        new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                // add the simulated drivetrain to the simulation field
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);
                
                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

                drive.driveSimulation = driveSimulation;
                AIRobotInSimulation.startOpponentRobotSimulations();
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                break;
        }
        drive.initalizeIntake();
        NamedCommands.registerCommand("Shoot", new SpamShootCommands(drive, true));
        NamedCommands.registerCommand("Intake Fuel", 
                Commands.runOnce(drive::intakeStart, drive)
        );
        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        SmartDashboard.putData("Depot Auto", new PathPlannerAuto("Depot Auto"));
        SmartDashboard.putData("Bump Auto", new PathPlannerAuto("Bump Auto"));
        SmartDashboard.putData("Align To Bump", Commands.runOnce(
                                ()->{alignToBump = new AlignToBump(drive); 
                                alignToBump.schedule();},
                        drive));
        
        configureButtonBindings();
        //setFrameKeyboardControl();
        
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    double slowSpeed = 0.4;
        double speedMult = 0.75;
        double rotMult = 0.65;
        AlignToHub alignToHub;
        AlignToBump alignToBump;
    private void configureButtonBindings() {
        double slowSpeed = 0.4;

                
        // new JoystickButton(controller, 4)
        // .whileTrue(DriveCommands.robotJoystickDrive(drive, 0, -slowSpeed, 0));
        // new JoystickButton(controller, 5)
        // .whileTrue(DriveCommands.robotJoystickDrive(drive, slowSpeed, 0, 0));
        // new JoystickButton(controller, 6)
        // .whileTrue(DriveCommands.robotJoystickDrive(drive, -slowSpeed, 0, 0));
        
        // Lock to 0° when A button is held
        // new JoystickButton(controller, 3)
        //         .whileTrue(DriveCommands.joystickDriveAtAngle(
        //                 drive, () -> controller.getY(), () -> controller.getX(), () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        // new JoystickButton(controller, 4).onTrue(Commands.runOnce(drive::stopWithX, drive));
        // new JoystickButton(controller, 1).onTrue(Commands.runOnce(drive::scoreAlgae, drive));
        // new JoystickButton(controller, 5).onTrue(Commands.runOnce(drive::spawnAlgae, drive));
        // new JoystickButton(controller, 6).onTrue(Commands.runOnce(drive::spawnCoral, drive));


        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
        ? () -> drive.resetOdometry(
                driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
        : () -> drive.resetOdometry(
                new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro

        if(controllerMode){
                if (DriverStation.getAlliance().equals(Alliance.Red)){
                        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> controller.getRawAxis(1)*speedMult, () -> controller.getRawAxis(0)*speedMult, () -> -controller.getRawAxis(4)*rotMult)
                );
                } else {
                        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getRawAxis(1)*speedMult, () -> -controller.getRawAxis(0)*speedMult, () -> -controller.getRawAxis(4)*rotMult)
                );
                }
            
            
            
            
            new Trigger(() -> controller.getRawAxis(/*change*/2) > 0.5)
                .onTrue(
                        Commands.runOnce(drive::intakeStart, drive)
                );
              new JoystickButton(controller, /*change*/5)
                .onTrue(
                        Commands.runOnce(drive::intakeStop, drive)
                );
                new JoystickButton(controller, /*change*/6)
                .onTrue(
                        Commands.runOnce(() -> {alignToHub = new AlignToHub(drive); alignToHub.schedule();}, drive)
                ).onFalse(Commands.runOnce(()->alignToHub.cancel()));
                new JoystickButton(controller, /*change*/4)
                .onTrue(
                        Commands.runOnce(
                                ()->{alignToBump = new AlignToBump(drive); 
                                alignToBump.schedule();},
                        drive)
                ).onFalse(Commands.runOnce(()->alignToBump.cancel()));
                new Trigger(() -> controller.getRawAxis(/*change*/3) > 0.5)
                .onTrue(//change to whiletrue for holding.
                            new SpamShootCommands(drive,false)
                );
            // fix this to a pov, povDown GenericHID
        //     new JoystickButton(controller, /*change*/2)
        //         .whileTrue(DriveCommands.toggleDrive().alongWith(Commands.run(() -> controller.setRumble(
        //                 RumbleType.kBothRumble, 0.5)))); //toggle Field Relative
        // new JoystickButton(controller, 2)
        //         .onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
        
       // Arena2026Rebuilt a = SimulatedArena.getInstance();

        //     new JoystickButton(controller, /*change*/3)
        //         .onTrue(
        //                 Commands.runOnce(drive::spawnFuel, drive)
        //         );

        //     new JoystickButton(controller, /*change*/5) 
        //         .whileTrue(DriveCommands.robotJoystickDrive(drive, 0, slowSpeed, 0));
                    
        //     new JoystickButton(controller, /*change*/6)
        //         .whileTrue(DriveCommands.robotJoystickDrive(drive, 0, -slowSpeed, 0));


        //     new Trigger(() -> controller.getRawAxis(/*change*/3) > 0.5)
        //         .onTrue(
        //                 Commands.runOnce(drive::scoreFuel, drive)
        //         );
            
                

            new JoystickButton(controller, 8) //press Rjoystick for reset gyro
                    .onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        } else {
            // if we aren't using controller
        //     drive.setDefaultCommand(DriveCommands.joystickDrive(
        //             drive, 
        //             () -> controller.getRawAxis(1) * speedMult, 
        //             () -> controller.getRawAxis(0) * speedMult, 
        //             () -> -controller.getRawAxis(2) * rotMult));
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                    drive, 
                    () -> controller.getRawAxis(1) * speedMult, 
                    () -> controller.getRawAxis(0) * speedMult, 
                    () -> -controller.getRawAxis(2) * rotMult)
        );
        //DriveCommands.updateCustomSpeedMult(1);

            Command goOverBump = DriveCommands.joystickDrive(drive,
                        () -> DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))?
                        (drive.getPose().getX()<4.626?speedMult:-speedMult):
                        (drive.getPose().getX()<11.915?speedMult:-speedMult),() -> 0, () -> 0);
        //         Command alignToBump = new AlignToBump2(drive);
                
        //     new JoystickButton(controller, 10).onTrue(alignToBump);
           // Commands.runOnce(()-> {DriveCommands.updateCustomSpeedMult(0.5);}, drive)
                //.alongWith(alignToBump)//.andThen(goOverBump)
        //     ).onFalse(
        //         //Commands.runOnce(()->alignToBump.cancel()).andThen(
        //        Commands.runOnce(
        //         ()-> {speedMult=1;}, drive));

            new JoystickButton(controller, 11).whileTrue(DriveCommands.toggleDrive());


            new JoystickButton(controller, 5)
            .whileTrue(DriveCommands.robotJoystickDrive(drive, slowSpeed, 0, 0));
            new JoystickButton(controller, 6)
            .whileTrue(DriveCommands.robotJoystickDrive(drive, -slowSpeed, 0, 0));

            new JoystickButton(controller, 1).onTrue(Commands.runOnce(drive::scoreFuel, drive));

            new JoystickButton(controller, 3)
                    .onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
                        new JoystickButton(controller, /*change*/7)
                .onTrue(
                        Commands.runOnce(drive::intakeStart, drive)
                );
            new JoystickButton(controller, 2).whileTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true).andThen(new AlignToHub(drive)));
        
        }
        controllerSim = new GenericHIDSim(controller);   
        }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));

        Logger.recordOutput("FieldSimulation/OpponentRobotPositions", AIRobotInSimulation.getOpponentRobotPoses());
        Logger.recordOutput(
                "FieldSimulation/AlliancePartnerRobotPositions", AIRobotInSimulation.getAlliancePartnerRobotPoses());
    }

    

    public void setFrameKeyboardControl(ConcurrentLinkedQueue<Runnable> queue){
        JLabel label = new JLabel("No input");
        label.setFont(new Font("Arial",Font.BOLD, 30));
        frame.setTitle("My JFrame Example");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(400, 300);
        frame.add(label);
        frame.setVisible(true);
        frame.addKeyListener(
                new KeyListener() {
                        @Override
                        public void keyPressed(KeyEvent e){
                                switch (e.getKeyCode()){
                                        case KeyEvent.VK_W:
                                        queue.add(() -> DriveCommands.joystickDrive(drive, () -> 0, () -> -1, () -> 0).schedule());
                                        break;
                                        case KeyEvent.VK_S:
                                        queue.add(() -> DriveCommands.joystickDrive(drive, () -> 0, () -> 1, () -> 0).schedule());
                                        break;
                                        case KeyEvent.VK_A:
                                        queue.add(() -> DriveCommands.joystickDrive(drive, () -> 1, () -> 0, () -> 0).schedule());
                                        break;
                                        case KeyEvent.VK_D:
                                        queue.add(() -> DriveCommands.joystickDrive(drive, () -> -1, () -> 0, () -> 0).schedule());
                                        break;
                                        case KeyEvent.VK_LEFT:
                                        queue.add(() -> DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 1).schedule());
                                        break;
                                        case KeyEvent.VK_RIGHT:
                                        queue.add(() -> DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> -1).schedule());
                                        break;
                                        case KeyEvent.VK_K:
                                                queue.add(() -> new SpamShootCommands(drive,false));
                                        break;
                                        case KeyEvent.VK_J:
                                                queue.add(drive::intakeStart);
                                        break;
                                        case KeyEvent.VK_L:
                                                queue.add(drive::intakeStop);
                                        break;
                                        case KeyEvent.VK_O:
                                                queue.add(() -> {alignToHub = new AlignToHub(drive); alignToHub.schedule();});
                                        case KeyEvent.VK_U:
                                                queue.add(() -> new AlignToBump(drive));
                                        break;
                                }
                                label.setText(KeyEvent.getKeyText(e.getKeyCode()));
                                controllerSim.notifyNewData();
                        }
                        @Override
                        public void keyReleased(KeyEvent e){
                                switch (e.getKeyCode()){
                                        case KeyEvent.VK_O:
                                                queue.add(() -> alignToHub.cancel());
                                        break;
                                }
                                DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0).schedule();
                                
                        }
                        @Override
                        public void keyTyped(KeyEvent e){

                        }
                }
        );
        
    }
}