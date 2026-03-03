// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.Map;
import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;


import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
    private final Boolean controllerMode = false;
    
    private final GenericHID controller = new GenericHID(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    private Command alignCommand = null;
    
  private enum BehaviorSelector
  {
    SHOOT,
    PASS
  }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
        arena.isActive(true);
        arena.setShouldRunClock(true);
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
         alignCommand = new SelectCommand<>(
      Map.ofEntries(
        Map.entry(BehaviorSelector.SHOOT, new AlignToHub(drive)),
        Map.entry(BehaviorSelector.PASS, new InstantCommand(() -> System.out.println("Selected PASS")))
      ),
      this::passOrShootSelector
    );
        // Configure the button bindings
        configureButtonBindings();

        drive.initalizeIntake();

    }

    double speedMult = 0.75;
    double rotMult = 0.65;

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        double slowSpeed = 0.4;

        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
        ? () -> drive.resetOdometry(
                driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
        : () -> drive.resetOdometry(
                new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro

        if(controllerMode){
            drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getRawAxis(1)*2, () -> -controller.getRawAxis(0)*2, () -> controller.getRawAxis(4))
                );
            
            
            
            new JoystickButton(controller, /*change*/1)
                .onTrue(
                        Commands.runOnce(drive::intakeStart, drive)
                );
            new JoystickButton(controller, /*change*/2)
                .onTrue(
                        Commands.runOnce(drive::intakeStop, drive)
                );
            // fix this to a pov, povDown GenericHID
            new POVButton(controller, /*change*/180)
                .whileTrue(DriveCommands.toggleDrive()); //toggle Field Relative

            new JoystickButton(controller, /*change*/3)
                .onTrue(
                        Commands.runOnce(drive::spawnFuel, drive)
                );

            new JoystickButton(controller, /*change*/5) 
                .whileTrue(DriveCommands.robotJoystickDrive(drive, 0, slowSpeed, 0));
                    
            new JoystickButton(controller, /*change*/6)
                .whileTrue(DriveCommands.robotJoystickDrive(drive, 0, -slowSpeed, 0));


            new Trigger(() -> controller.getRawAxis(/*change*/3) > 0.5)
                .onTrue(
                        Commands.runOnce(drive::scoreFuel, drive)
                );
            
                

            new JoystickButton(controller, 9) //press Rjoystick for reset gyro
                    .onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        } else {
            // if we aren't using controller
            drive.setDefaultCommand(DriveCommands.joystickDrive(
                    drive, 
                    () -> controller.getRawAxis(1) * speedMult, 
                    () -> controller.getRawAxis(0) * speedMult, 
                    () -> -controller.getRawAxis(2) * rotMult));
            
            new JoystickButton(controller, 10).onTrue(DriveCommands.toggleDrive());


            new JoystickButton(controller, 5)
            .whileTrue(DriveCommands.robotJoystickDrive(drive, slowSpeed, 0, 0));
            new JoystickButton(controller, 6)
            .whileTrue(DriveCommands.robotJoystickDrive(drive, -slowSpeed, 0, 0));

            new JoystickButton(controller, 1).onTrue(Commands.runOnce(drive::scoreFuel));

            new JoystickButton(controller, 3)
                    .onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
                        new JoystickButton(controller, /*change*/7)
                .onTrue(
                        Commands.runOnce(drive::intakeStart, drive)
                );
            new JoystickButton(controller, 2)
            .whileTrue(
                Commands.runOnce(resetGyro, drive)
                .ignoringDisable(true)
                .andThen(new InstantCommand(()->{AlignToHub.setGlobalAngleOffsetRad0();}))
                .andThen(alignCommand)
            );

        }
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

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
      public BehaviorSelector passOrShootSelector() {
        boolean isBlue = DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue));
    double threshold = isBlue ? 4 : 16.54 - 4;
    if((drive.getPose().getX()>threshold)==isBlue)
    {
      return BehaviorSelector.PASS;
    }
    else
    {
      return BehaviorSelector.SHOOT;
    }
  }
}