package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AlignToBump2 extends Command{
    PIDController rotController = new PIDController(0.03, 0, 0);
    Drive m_drive;
    GenericHID m_controller;
    double error;
    double speedMult;
    public AlignToBump2(Drive drive, GenericHID controller) {
        rotController.setSetpoint(0);//This makes the robot face 0 degrees
        rotController.enableContinuousInput(-180, 180);
        m_drive = drive;
        m_controller = controller;
    }
    @Override
    public void initialize() {
        // SmartDashboard.putData("AlignToBumpRotController", rotController);
        // SmartDashboard.putNumber("AlignToBumpError", error);
    }
    @Override
    public void execute() {
        error = m_drive.getPose().getRotation().getDegrees()-0;//Get the error between current angle and desired angle.
        double rotSpeed = rotController.calculate(error);
        //Normally, speedMult will be a constant taken from the normal speed of the joystick.
        //This feature will allow the driver to drive around while rotating. 
        //You can also slow down the robot for microadjustments.
        DriveCommands.joystickDrive(m_drive, () -> -m_controller.getRawAxis(1)*speedMult, () -> -m_controller.getRawAxis(0)*speedMult, ()->rotSpeed).execute();
    }
    public void setSpeedMult(double mult){
        speedMult = mult;
    }
    @Override
    public void end(boolean interrupted) {
        if (!interrupted){
            goOverBump().schedule();
        }
    }
    @Override 
    public boolean isFinished(){
        //return false;
        return Math.abs(error)<1;
    }
    public Command goOverBump(){
        double odomError = 0.3;
        double xPos = m_drive.getPose().getX();
        double bumpSpeed = 0.7;//This is percentage of the power needed, so it would be (needed speed / max speed).
        if (xPos<8.256) { //If the robot is on the blue side of the field
                if (xPos<4.626){
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
