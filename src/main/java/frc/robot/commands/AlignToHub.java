// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToHub extends Command {
  /** Creates a new AlignToHub. */
  Drive m_drive;
  PIDController xController = new PIDController(1, 0, 0);
  PIDController yController = new PIDController(1, 0, 0);
  PIDController rotController = new PIDController(0.03, 0, 0.001);

  private final GenericHID controller = new GenericHID(0);
  double speedMult = 0.6;
  double globalTargetAngle;

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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_drive.getPose();
    double[] errors = CalculateHubPID(currentPose);
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
    public double[] CalculateHubPID(Pose2d pose) {
        double robotX = pose.getX();
		    double robotY = pose.getY();

        double[] errors = new double[3];
        double radius = 2.75;
        double hubY = 4.03; // meters
        double hubXBlue = 4.63;
        double hubXRed = 11.92;
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


        double errorX = distanceX * ( (distance - radius) / distance );
        double errorY = distanceY * ( (distance - radius) / distance );
        double targetAngle = Math.signum(distanceY) * Math.toDegrees(Math.acos(distanceX / distance));
        globalTargetAngle = targetAngle;
        double errorAngle = targetAngle - pose.getRotation().getDegrees();

        double shooterOffset = .46; //meters
        double initialEjectionVelocity = 6.5; //m/s
        double ejectionAngle = 68; //deg

        double timeOfFlight = (radius - shooterOffset)/(initialEjectionVelocity*Math.cos(Math.toRadians(ejectionAngle)));
        double initialVelocityRobotRelative = m_drive.getChassisSpeeds().vyMetersPerSecond;

        double angleOffset = Math.toDegrees(Math.atan(initialVelocityRobotRelative * timeOfFlight/radius));
        SmartDashboard.putNumber("AlignToHub/ErrorX", errors[0]);

		    errors[0] = errorX;
		    errors[1] = errorY;
		    errors[2] = errorAngle - (1*angleOffset);


       SmartDashboard.putNumber("AlignToHub/TargetAngle",targetAngle);

        SmartDashboard.putNumber("AlignToHub/ErrorX", errors[0]);
        SmartDashboard.putNumber("AlignToHub/ErrorY", errors[1]);
        SmartDashboard.putNumber("AlignToHub/ErrorAngle", errors[2]);

       SmartDashboard.putNumber("AlignToHub/RobotX", robotX);
        SmartDashboard.putNumber("AlignToHub/RobotY", robotY);
        SmartDashboard.putNumber("AlignToHub/RobotAngle", pose.getRotation().getDegrees());
        return errors;
    }
}

