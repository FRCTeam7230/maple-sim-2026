package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class SpamShootCommands extends Command{
    Drive robotDrive;
    Timer timer = new Timer();
    public SpamShootCommands(Drive drive) {
        robotDrive = drive;
        addRequirements(drive);
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
        return !robotDrive.hasFuelInIntake();
    }
    @Override
    public void end(boolean interrupted){

    }
}
