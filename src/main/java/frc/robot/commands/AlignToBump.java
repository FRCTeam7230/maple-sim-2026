package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class AlignToBump extends Command{
    PIDController rotController = new PIDController(0.03, 0, 0);
    Drive m_drive;
    double currentAngle;
    double rotSpeed;

    boolean readyToDriveOver;
    Command driveCommand;
    double a = 0;
    public AlignToBump(Drive drive) {
        rotController.setSetpoint(0);//This makes the robot face 0 degrees.
        rotController.enableContinuousInput(-180, 180);
        m_drive = drive;
        //addRequirements(drive);
    }
    @Override
    public void initialize() {
        SmartDashboard.putData("AlignToBump/rotController", rotController);
        readyToDriveOver = false;
    }
    @Override
    public void execute() {
        //a++;
        //error = ;//Get the error between current angle and desired angle.
        currentAngle = m_drive.getPose().getRotation().getDegrees();
        if (currentAngle>-90&&currentAngle<90){
            rotController.setSetpoint(0);
        } else {
            rotController.setSetpoint(180);
        }
        rotSpeed = rotController.calculate(currentAngle);
        //SmartDashboard.putNumber("AlignToBump/Current a", a);
        SmartDashboard.putNumber("AlignToBump/Current Angle", currentAngle);
        if (driveCommand!=null){
        SmartDashboard.putBoolean("AlignToBump/Going over bump", driveCommand.isScheduled());}
        SmartDashboard.putNumber("AlignToBump/Error", rotController.getError());
        SmartDashboard.putNumber("AlignToBump/Rotation Speed", rotSpeed);
        if (!readyToDriveOver){
            DriveCommands.joystickDrive(m_drive, ()->0, ()->0, ()->rotSpeed).schedule();
            if (Math.abs(currentAngle-rotController.getSetpoint())<2&&Math.abs(m_drive.getTurnRate())<0.02){//change turn rate to 1 deg.
                driveCommand = goOverBump();
                driveCommand.schedule();
                readyToDriveOver = true;
            }
        }
    }
    @Override
    public void end(boolean interrupted) {
        if (driveCommand!=null){
            driveCommand.cancel();
        }
    }
    @Override 
    public boolean isFinished(){
        return false;
        //return readyToDriveOver&&!driveCommand.isScheduled();
    }
    public Command goOverBump(){
        double odomError = 0.05;
        double xPos = m_drive.getPose().getX();
        double bumpSpeed = 1;//This is percentage of the power needed, so it would be (needed speed / max speed).
        if (xPos<8.256) { //If the robot is on the blue side of the field
                if (xPos<4.626){
                        // return Commands.run(()->m_drive.drive(
                        //         -bumpSpeed,0, 0,true))
                        //         .until(()->m_drive.getPose().getX()>4.626+0.5588+odomError);//don't ask why this increases x value.
                        return DriveCommands.joystickDrive(m_drive,
                                ()-> bumpSpeed,() -> 0, () -> 0).until(()->m_drive.getPose().getX()>4.626+0.5588+odomError);
                } else {
                        return DriveCommands.joystickDrive(m_drive,
                                ()-> -bumpSpeed,() -> 0, () -> 0).until(()->m_drive.getPose().getX()<4.626-0.5588-odomError);
                }
        } else {
            if (xPos<11.915){
                return DriveCommands.joystickDrive(m_drive,
                        ()-> bumpSpeed,() -> 0, () -> 0).until(()->m_drive.getPose().getX()>11.915+0.5588+odomError);
            } else {
                return DriveCommands.joystickDrive(m_drive,
                        ()-> -bumpSpeed,() -> 0, () -> 0).until(()->m_drive.getPose().getX()<11.915-0.5588-odomError);
            }
        }
    }
}
