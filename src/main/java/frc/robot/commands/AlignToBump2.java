package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class AlignToBump2 extends Command{//This is the better edition
    PIDController rotController = new PIDController(0.04, 0, 0);
    Drive m_drive;
    double currentAngle;
    double rotSpeed;

    int drivingOverTheBumpDirectionMode;
    double bumpSpeed = 1;//This is percentage of the power needed, so it would be (needed speed / max speed).
    double odomError = 0.3;

    double goalAngle = 38.412;//Angles 38.412, 141.588, -38.412, -141.588 all work for align to bump.
    public AlignToBump2(Drive drive) {
        rotController.setSetpoint(0);//This makes the robot face 0 degrees.
        rotController.enableContinuousInput(-180, 180);
        m_drive = drive;
        //addRequirements(m_drive);//i removed this when testing it with a button.
    }
    @Override
    public void initialize() {
        //SmartDashboard.putData("AlignToBump/rotController", rotController);
        drivingOverTheBumpDirectionMode = 0;
    }
    @Override
    public void execute() {
       // SmartDashboard.putNumber("AlignToBump/driveState", drivingOverTheBumpDirectionMode);
        switch (drivingOverTheBumpDirectionMode){
            case 1:
            case 3:
                m_drive.drive(bumpSpeed,0, 0,true); 
            break;
            case 2: case 4:
                m_drive.drive(-bumpSpeed,0, 0,true); 
            break;
            default:
                currentAngle = m_drive.getPose().getRotation().getDegrees();
                if (currentAngle>goalAngle-90&&currentAngle<goalAngle+90){
                    rotController.setSetpoint(goalAngle);
                } else {
                    rotController.setSetpoint(goalAngle+180);
                }
                rotSpeed = rotController.calculate(currentAngle);

                SmartDashboard.putNumber("AlignToBump/Current Angle", currentAngle);
                SmartDashboard.putNumber("AlignToBump/Error", rotController.getError());
                SmartDashboard.putNumber("AlignToBump/Rotation Speed", rotSpeed);
                SmartDashboard.putNumber("AlignToBump/Turn Rate", Math.abs(m_drive.getTurnRate()));

                m_drive.drive(0, 0, rotSpeed, true);
                if (Math.abs(rotController.getError())<2&&Math.abs(m_drive.getTurnRate())<0.018){//change turn rate to 1 deg.
                    drivingOverTheBumpDirectionMode = findDrivingDirection();//2, 0.02
                }
        }
    }
    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
    @Override 
    public boolean isFinished(){
        switch (drivingOverTheBumpDirectionMode){
            case 1:
                return m_drive.getPose().getX()>4.626+0.5588+odomError;
            case 2:
                return m_drive.getPose().getX()<4.626-0.5588-odomError;
            case 3:
                return m_drive.getPose().getX()>11.915+0.5588+odomError;
            case 4:
                return m_drive.getPose().getX()<11.915-0.5588-odomError;
            default:
                return false;
        }
    }
    public int findDrivingDirection(){
        double xPos = m_drive.getPose().getX();//This cannnot be updated in periodic
        if (xPos<8.256) { //If the robot is on the blue side of the field
                if (xPos<4.626){
                    return 1;
                } else {
                    return 2;
                }
        } else {
            if (xPos<11.915){
                return 3;
            } else {
                return 4;
            }
        }
    }
    
}
