package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class SpamShootCommands extends Command{
    Drive robotDrive;
    Timer timer = new Timer();
    boolean autoMode;

    double timeRemaning = DriverStation.getMatchTime();
    
    public SpamShootCommands(Drive drive, boolean autoMode) {
        robotDrive = drive;
        this.autoMode = autoMode;
        //addRequirements(drive);
    }
    @Override 
    public void initialize(){
        timer.reset();
        timer.start();
    }
    @Override
    public void execute(){
        if(timer.hasElapsed(0.1)){
            robotDrive.scoreFuel();
            timer.reset();
        }
    }
    @Override
    public boolean isFinished(){
        return !robotDrive.hasFuelInIntake()||DriverStation.getMatchTime()<7&&DriverStation.getMatchTime()>0;
    }
    @Override
    public void end(boolean interrupted){

    }
}
