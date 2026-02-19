package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AlignToBump extends Command{
    PIDController rotController = new PIDController(0.03, 0, 0);
    Drive drive;
    double error;
    public AlignToBump(Drive drive) {
        rotController.setSetpoint(0);//This makes the robot face 0 degrees.
        rotController.enableContinuousInput(-180, 180);
        this.drive = drive;
    }
    @Override
    public void initialize() {
        SmartDashboard.putData("AlignToBump/rotController", rotController);
        SmartDashboard.putNumber("AlignToBump/Error", error);
    }
    @Override
    public void execute() {
        error = drive.getRotation().getDegrees()-0;//Get the error between current angle and desired angle.
        double rotSpeed = rotController.calculate(error);
        DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> rotSpeed).withTimeout(2).execute();
    }
    @Override
    public void end(boolean interrupted) {

    }

}
