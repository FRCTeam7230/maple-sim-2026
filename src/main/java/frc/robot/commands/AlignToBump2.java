package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class AlignToBump2 extends Command{//This is the better edition
    /** PID Controller for rotation of robot(goal orientation is diagonal) */
    PIDController rotController = new PIDController(0.04, 0, 0.001);
    Drive m_drive;
    double currentAngle;
    double rotSpeed;

    int drivingOverTheBumpDirectionMode;
    double bumpSpeed = 1.7/4.8;//This is percentage of the power needed, so it would be (needed speed / max speed).
    
    double odomError = 0.3;

    double goalAngle = 38.412;//Angles 38.412, 141.588, -38.412, -141.588 all work for align to bump.

    public AlignToBump2(Drive drive) {
        rotController.setSetpoint(0);//This makes the robot face 0 degrees.
        rotController.enableContinuousInput(-180, 180);
        m_drive = drive;
        bumpSpeed = 4.8;
        addRequirements(m_drive);//i removed this when testing it with a button.
    }
     double angle1 = goalAngle;//38.412
    double angle2 = (180-goalAngle);//141.588
    double angle3 = (180+goalAngle);//-38.412
    double angle4 = (360-goalAngle);//-141.588
    @Override
    public void initialize() {
        //SmartDashboard.putData("AlignToBump/rotController", rotController);
        //Makes the angle in a range of -180 to 180
        // if (currentAngle > 180) {
        // currentAngle -= 360;
        // }
        // else if (currentAngle < -180) {
        // currentAngle += 360;
        // }

        //This makes sure the robot angle between 0 to 360, so we can use it to determine the current robot heading
        drivingOverTheBumpDirectionMode = 0;
        currentAngle = m_drive.getPose().getRotation().getDegrees();
        while (currentAngle > 360 || currentAngle<0) {
            if(currentAngle>360){
            currentAngle -= 360;
            }
            else{
            currentAngle += 360;
            }
            
        }
        // if (currentAngle>angle1 && currentAngle<angle3){
        //     rotController.setSetpoint(angle1);
        // } 
            // if (Math.abs(Math.abs(currentAngle)-angle1)<=Math.abs(Math.abs(currentAngle)-angle2)){
            //     rotController.setSetpoint(angle1);

            // }
            // else {
        //     rotController.setSetpoint(angle3);
        // }{
        
        // if (currentAngle>angle2-(angle2-angle1)/2&&currentAngle<angle2+(angle3-angle2)/2){
        //     rotController.setSetpoint(angle2);
        // } else if (currentAngle>angle3-(angle3-angle2)/2&&currentAngle<angle3+(angle4-angle3)/2){
        //     rotController.setSetpoint(angle3);
        // } else if (currentAngle>angle4-(angle4-angle3)/2&&currentAngle<angle4+(angle1-angle4+360)/2){
        //     rotController.setSetpoint(angle4);
        // } else {
        //     rotController.setSetpoint(angle1);
        // }

        //used to find closest angle and sets as target angle
        if (currentAngle>90&&currentAngle<180){
            rotController.setSetpoint(angle2);
        } else if (currentAngle>180&&currentAngle<270){
            rotController.setSetpoint(angle3);
        } else if (currentAngle>270&&currentAngle<360){
            rotController.setSetpoint(angle4);
        } else if (currentAngle>0&&currentAngle<90){
            rotController.setSetpoint(angle1);
        } else {
            //Add a breakpoint here if you want to check if currentAngle is returning weird values.
        }
        SmartDashboard.putNumber("AlignToBump/Target Angle", rotController.getSetpoint());
           
    }
   
    @Override
    public void execute() {
       // SmartDashboard.putNumber("AlignToBump/driveState", drivingOverTheBumpDirectionMode);

       //Drives to position
        switch (drivingOverTheBumpDirectionMode){
            case 1:
            case 3:
                m_drive.drive(bumpSpeed,0, 0,true); 
            break;
            case 2: case 4:
                m_drive.drive(-bumpSpeed,0, 0,true); 
            break;
            default:
                currentAngle = m_drive.getPose().getRotation().getDegrees()%360;
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
    //stops the robot at the end of the command
    public void end(boolean interrupted) {
        m_drive.stop();
    }
    @Override 
    public boolean isFinished(){
        //checks if it is finished by checking if robot has driven over bump
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
    /** Returns the driving direction based on the robot's current x position */
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
