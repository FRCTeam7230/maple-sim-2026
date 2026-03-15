// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToHub extends Command {
  /* Creates a new AlignToHub. */

  Drive m_drive;
  PIDController xController = new PIDController(1, 0, 0);
  PIDController yController = new PIDController(1, 0, 0);
  PIDController rotController = new PIDController(0.07, 0, 0.002);

  private final GenericHID controller = new GenericHID(0);
  private final Timer timer = new Timer();
  double speedMult = 0.5;
  double globalTargetAngle;
  
  double lastTime;
  double radalOffset;

  Command drivecommand = null;
  public AlignToHub(Drive drive) {
    m_drive = drive;
    xController.setSetpoint(0);
    yController.setSetpoint(0);
    rotController.setSetpoint(0);
    rotController.enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    radalOffset = 0;
    lastTime = timer.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_drive.getPose();
    double deltaTime = timer.get() - lastTime;
    lastTime = timer.get();
    radalOffset = controller.getRawAxis(1)*deltaTime*30;
    double[] errors = CalculateHubPID(currentPose, radalOffset);
    double xSpeed = xController.calculate(errors[0]);
    double ySpeed = yController.calculate(errors[1]);
    double rotSpeed = Math.max(Math.min(rotController.calculate(errors[2]),1.5), -1.5);
    SmartDashboard.putNumber("Rotation delivered", rotSpeed);
    
    drivecommand = DriveCommands.joystickDrive(
      m_drive,
      ()->{return -(xSpeed + (controller.getRawAxis(0) * speedMult * Math.sin(Math.toRadians(-globalTargetAngle))));},
      ()->{return -(ySpeed  + (controller.getRawAxis(0) * speedMult * Math.cos(Math.toRadians(-globalTargetAngle))));},
      ()->{return -rotSpeed;});
    drivecommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_drive.stop();
    drivecommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //constants
  double Radius = 2.75;
  double hubY = 4.03; // meters
  double hubXBlue = 4.63;
  double hubXRed = 11.92;
  double hubHeight = 1.8288; // meters
  double shooterOffset = .46; //meters
  double initialEjectionVelocityBeforeOffset = 7; //m/s
  double initialEjectionVelocityAfterOffset; //m/s
  double ejectionAngle = 67; //deg
  double radiusToleranceForward = -0.6; //meters
  double radiusToleranceBackward = 3; //meters
  double DistMulti = 1.1; //multi, minimum should be 1.1

  //declarations for scope
  double timeOfFlight;
  double zInitialVelocityRobotRelative = 0;
  public static double globalAngleOffsetRad = 0; // when commands ends need to set this to 0
  public static void setGlobalAngleOffsetRad0() {
    globalAngleOffsetRad = 0;
  }


  //sx constants and delcarations
  double x0 = shooterOffset;
  double vx0;
  double ax = 0;

  /*sy constants and delcarations. not used
  double y0 = 0.38; //initial height.
  double vy0 = initialEjectionVelocity*Math.sin(Math.toRadians(ejectionAngle));
  double ay = -9.81;
  */

  //sz constants and delcarations
  double z0 = 0;
  double az = 0; //ignoring any horizontal accelerations for now

  public double[] CalculateHubPID(Pose2d pose, double radalOffset) {
    //Math: if we have exit velocity for our distance, we can do (Vcos(theta)+robotVelocity)/cos(theta) = exit velocity at the angle
    // V is our exit velocity withou the consideration of robot velocity, away from hub is positive direction for velocity
      double robotX = pose.getX();
		  double robotY = pose.getY();

      

      //error calculations
      double[] errors = new double[3];
        
      double hubX;
      if (DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
          hubX = hubXBlue;
          
      }
      else {
          hubX = hubXRed;
      }

      double distanceX = hubX - robotX;
      double distanceY = hubY - robotY;
      double distance = Math.sqrt( Math.pow( distanceX, 2) + Math.pow( distanceY, 2) );

      double radius = distance + radalOffset;
      double targetVelocityX = Math.sqrt(
        (9.81*Math.pow(distance*DistMulti, 2))
        /(2*Math.tan(Math.toRadians(ejectionAngle))*distance*DistMulti - (hubHeight-0.677))
        );
      double velocityCancel = m_drive.getChassisSpeeds().vxMetersPerSecond;
      if(velocityCancel>1.35){
        velocityCancel *= 0.9;
      } else if (velocityCancel>0.85){
        velocityCancel *= 0.85;
      } else if(velocityCancel<=0.85){
        velocityCancel *= 1.2;
      }
      double targetVelocity =  targetVelocityX / Math.cos(Math.toRadians(ejectionAngle)) - velocityCancel;

      SmartDashboard.putNumber("AlignToHub/targetVelocity", targetVelocity);
      SmartDashboard.putNumber("AlignToHub/VelocityofRobotX", m_drive.getChassisSpeeds().vxMetersPerSecond);

      if((radius-Radius)>radiusToleranceBackward){
        radius = Radius + radiusToleranceBackward;
      } else if((radius-Radius)<radiusToleranceForward){
        radius = Radius + radiusToleranceForward;
      }

      SmartDashboard.putNumber("AlignToHub/radius", radius);

      double errorX = distanceX * ( (distance - radius) / distance );
      double errorY = distanceY * ( (distance - radius) / distance );

      double targetAngle = Math.signum(distanceY) * Math.toDegrees(Math.acos(distanceX / distance));
      globalTargetAngle = targetAngle;
      double errorAngle = targetAngle - pose.getRotation().getDegrees();

      //angle offset calculations
      initialEjectionVelocityAfterOffset = initialEjectionVelocityBeforeOffset + Math.abs(0.3*zInitialVelocityRobotRelative);
      initialEjectionVelocityAfterOffset += targetVelocity-6.58;
      m_drive.setInitialVelocity(initialEjectionVelocityAfterOffset);
      vx0 = initialEjectionVelocityBeforeOffset*Math.cos(Math.toRadians(ejectionAngle));

      timeOfFlight = (radius - x0)/(initialEjectionVelocityBeforeOffset*Math.cos(Math.toRadians(ejectionAngle)));
      zInitialVelocityRobotRelative = m_drive.getChassisSpeeds().vyMetersPerSecond;

      //super old offset calc
      //double angleOffsetRad = (Math.atan(zInitialVelocityRobotRelative * timeOfFlight/radius)); 

      //new offset calc
      
      double t1 = timeOfFlight; //time of normal trajectory from shooter to hub
      double theta2Rad = (Math.atan( (sz(t1)) / (radius - shooterOffset) )); //rad

      double x0New = shooterOffset * Math.cos((theta2Rad));
      double t2 = (radius - x0New) / ( (vx0 * Math.cos((theta2Rad))) + (zInitialVelocityRobotRelative * Math.sin(theta2Rad)) );
      double theta3Rad = (Math.atan( (szNew(t2,theta2Rad)) / ( (sxNew(t2,theta2Rad)) - x0New) )); //rad

      double angleOffsetRad = theta2Rad + theta3Rad; // rad. idk if add or sub. pretty sure add
      globalAngleOffsetRad = angleOffsetRad;

		  errors[0] = errorX;
		  errors[1] = errorY;
		  errors[2] = errorAngle - Math.toDegrees(angleOffsetRad); //deg

      //offset calc publisher 
      
      SmartDashboard.putNumber("AlignToHub/theta2Deg", Math.toDegrees(theta2Rad));
      SmartDashboard.putNumber("AlignToHub/theta3Deg", Math.toDegrees(theta3Rad));
      SmartDashboard.putNumber("AlignToHub/angleOffset", Math.toDegrees(angleOffsetRad));
      SmartDashboard.putNumber("AlignToHub/zInitialVelocity", zInitialVelocityRobotRelative);
      SmartDashboard.putNumber("AlignToHub/initalEjectionVelocity", initialEjectionVelocityAfterOffset);
      
      //error publisher
      SmartDashboard.putNumber("AlignToHub/ErrorX", errors[0]);
      SmartDashboard.putNumber("AlignToHub/TargetAngle",targetAngle);
      SmartDashboard.putNumber("AlignToHub/ErrorX", errors[0]);
      SmartDashboard.putNumber("AlignToHub/ErrorY", errors[1]);
      SmartDashboard.putNumber("AlignToHub/ErrorAngle", errors[2]);

      //robot pos publisher
      SmartDashboard.putNumber("AlignToHub/RobotX", robotX);
      SmartDashboard.putNumber("AlignToHub/RobotY", robotY);
      SmartDashboard.putNumber("AlignToHub/RobotAngle", pose.getRotation().getDegrees());
      SmartDashboard.putBoolean("AlignToHub/Alliance", DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue)));

      return errors;
    }
    ///* 
    private double sx(double t) {
      return x0 + vx0*t + .5*ax*Math.pow(t,2);
    }
    
    private double sz(double t) {
      return z0 + zInitialVelocityRobotRelative*t + .5*az*Math.pow(t,2);
    }
      //*/
    private double sxNew(double t, double theta) { //angle in rad
      return sx(t)*Math.cos((theta)) + sz(t)*Math.sin((theta)); //applies rotation about y-axis by angle theta
    }
    private double szNew(double t, double theta) { //angle in rad
      return sz(t)*Math.cos((theta)) - sx(t)*Math.sin((theta)); //applies rotation about y-axis by angle theta
    }
    /* //not used
    private double sy(double t) { 
      return y0 + vy0*t + .5*ay*Math.pow(t,2);
    }*/
    /*//try these later but probably dont need to - NEED TO MAKE globalAngleOffsetRad = 0 WHEN ALIGN COMMAND ENDS --> CHECK
    private double sx(double t) {
      return x0 + (Math.sin(globalAngleOffsetRad)*zInitialVelocityRobotRelative + vx0)*t + .5*ax*Math.pow(t,2);
    }
    private double sz(double t) {
      return z0 + zInitialVelocityRobotRelative*t + .5*az*Math.pow(t,2);
    }
    */

}

