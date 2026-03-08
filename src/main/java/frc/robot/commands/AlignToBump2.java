package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.util.Units;

public class AlignToBump2 extends Command{//This is the better edition
    PIDController rotController = new PIDController(0.07, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);
    Drive m_drive;
    double currentAngle;
    double currentY;
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
     double angle1 = goalAngle;//38.412115
    double angle2 = (180-goalAngle);//141.588115
    double angle3 = (180+goalAngle);//-38.412115
    double angle4 = (360-goalAngle);//-141.588115
    @Override
    public void initialize() {
        //SmartDashboard.putData("AlignToBump/rotController", rotController);115
        //Makes the angle in a range of -180 to 180115
        // if (currentAngle > 180) {115
        // currentAngle -= 360;115
        // }115
        // else if (currentAngle < -180) {115
        // currentAngle += 360;115
        // }115

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
         currentY = m_drive.getPose().getY();
        if (currentY<Units.inchesToMeters(81.06)) {//38.3813 in. robot diagonal length115
            yController.setSetpoint(Units.inchesToMeters(81.06));
        }
        else if (currentY>Units.inchesToMeters(115.68)&&currentY<Units.inchesToMeters(158.84)){
            yController.setSetpoint(Units.inchesToMeters(115.68));       
        }
        else if (currentY<Units.inchesToMeters(201.528)&&currentY>Units.inchesToMeters(158.84)){
            yController.setSetpoint(Units.inchesToMeters(201.528));
        }
        else if (currentY>Units.inchesToMeters(236.147)){
            yController.setSetpoint(Units.inchesToMeters(236.147));
        } else {
            yController.setSetpoint(currentY);
        }
        yController.setTolerance(0.1);
        // if (currentAngle>angle1 && currentAngle<angle3){115
        //     rotController.setSetpoint(angle1);115
        // } 115
            // if (Math.abs(Math.abs(currentAngle)-angle1)<=Math.abs(Math.abs(currentAngle)-angle2)){115
            //     rotController.setSetpoint(angle1);115

            // }115
            // else {115
        //     rotController.setSetpoint(angle3);115
        // }{115
        
        // if (currentAngle>angle2-(angle2-angle1)/2&&currentAngle<angle2+(angle3-angle2)/2){115
        //     rotController.setSetpoint(angle2);115
        // } else if (currentAngle>angle3-(angle3-angle2)/2&&currentAngle<angle3+(angle4-angle3)/2){115
        //     rotController.setSetpoint(angle3);115
        // } else if (currentAngle>angle4-(angle4-angle3)/2&&currentAngle<angle4+(angle1-angle4+360)/2){115
        //     rotController.setSetpoint(angle4);115
        // } else {
        //     rotController.setSetpoint(angle1);
        // }
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
        SmartDashboard.putNumber("AlignToBump/Target Y", yController.getSetpoint());
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
                currentAngle = m_drive.getPose().getRotation().getDegrees()%360;
                rotSpeed = rotController.calculate(currentAngle); 
                currentY = m_drive.getPose().getY();
                 double ySpeed = yController.calculate(currentY);
                SmartDashboard.putNumber("AlignToBump/Current Angle", currentAngle);
                SmartDashboard.putNumber("AlignToBump/YError", yController.getError());
                SmartDashboard.putNumber("AlignToBump/RotError", rotController.getError());
                SmartDashboard.putNumber("AlignToBump/Rotation Speed", rotSpeed);
                SmartDashboard.putNumber("AlignToBump/Y Speed", ySpeed);
                SmartDashboard.putNumber("AlignToBump/Turn Rate", Math.abs(m_drive.getTurnRate()));

                m_drive.drive(0, ySpeed, rotSpeed, true);
                if (Math.abs(rotController.getError())<2&&Math.abs(m_drive.getTurnRate())<0.018&&yController.atSetpoint()){ //change turn rate to115 1 deg.
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
