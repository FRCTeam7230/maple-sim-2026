// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double shootRate = 3;
  public static final double climbTime = 5;
  

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ControllerConstants
  {
    //Button configurations for the XBox controller
    public static final int kButton1 = 1; //A
    public static final int kButton2 = 2; //B
    public static final int kButton3 = 3; //X
    public static final int kButton4 = 4; //Y
    public static final int kButton5 = 5; //LB, left button
    public static final int kButton6 = 6; //RB, right button
    public static final int kButton7 = 7; //Screenshare button, dont use and back button     not used
    public static final int kButton8 = 8; //Menu button, probably dont use 
    public static final int kButton9 = 9; //Pressing down left joystick DO NOT USE
    public static final int kButton10 = 10; //Pressing down right joystick DO NOT USE

    /**
     * Setting the numbers of the povs to literally anything else will probably break everything 
     */
    public static final int pov0 = 0; //up
    public static final int pov45 = 45; //up right
    public static final int pov90 = 90; //right
    public static final int pov135 = 135; //down right
    public static final int pov180 = 180; //down
    public static final int pov225 = 225; //down left
    public static final int pov270 = 270; //left
    public static final int pov315 = 315; //up left

    public static final int leftTrigger = -2; //LT, left trigger 
    public static final int rightTrigger = -3; //RT, right trigger

    public static final int leftStick_XAXIS = 0;
    public static final int leftStick_YAXIS = 1;
    public static final int rightStick_XAXIS = 4;
    public static final int rightStick_YAXIS = 5;

    // Xbox controller mappings
    /** A button */
    public static final int INTAKE_DOWN = kButton1;
    /** B button */
    public static final int ROBOT_RELATIVE = kButton2;
    /** X button */
    public static final int BRAKE_BUTTON = kButton3;
    /** Y button */
    public static final int CLIMBUP = 7;
    
    public static final int CLIMBDOWN = 8;
    /** Left button */
    public static final int ALIGN_HUB = kButton5;
    /** Right button */
    public static final int SHOOT_HUB = kButton6;
    /** Menu button */
    public static final int ZERO_HEADING_BUTTON = kButton8;

    /** Left trigger, axis 2 */
    public static final int ALIGN_TRENCH = leftTrigger;
    /** Right trigger, axis 3 */
    public static final int SPIN_INTAKE = rightTrigger;
    
    // XBox movement mappings
    public static final int MOVE_XAXIS = leftStick_XAXIS;
    public static final int MOVE_YAXIS = leftStick_YAXIS;
    public static final int MOVE_ZAXIS = rightStick_XAXIS;
  }

  public static class ElevatorConstants
  {  
    public static final double kElevatorKp = 5;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;  
    public static final double kElevatorkS = 0.0; // volts (V)
    public static final double kElevatorkG = 0.762; // volts (V)
    public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))
  
    public static final double kElevatorGearing = 10.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kCarriageMass = 4.0; // kg

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0.0;
    //public static final double kMaxElevatorHeightMeters = 10.25;
    public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(72);

    public static final double kRotationToMeters = kElevatorDrumRadius * 2 * Math.PI;
    public static final double kRPMtoMPS = (kElevatorDrumRadius * 2 * Math.PI) / 60;
    public static final double kElevatorMaxVelocity = 3.5;
    public static final double kElevatorMaxAcceleration = 2.5;
  }

  public static final class ElevatorSimConstants 
  {
    public static final int kMotorPort = 0;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;
    public static final int kJoystickPort = 0;
  
    public static final double kElevatorKp = 0.75;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;
  
    public static final double kElevatorMaxV = 10.0; // volts (V)
    public static final double kElevatorkS = 0.0; // volts (V)
    public static final double kElevatorkG = 0.62; // volts (V)
    public static final double kElevatorkV = 3.9; // volts (V)
    public static final double kElevatorkA = 0.06; // volts (V)
  
    public static final double kElevatorGearing = 5.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(1.0);
    public static final double kCarriageMass = Units.lbsToKilograms(12); // kg
  
    public static final double kSetpointMeters = Units.inchesToMeters(42.875);
    public static final double kLowerkSetpointMeters = Units.inchesToMeters(15);
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(50);
  
    // distance per pulse = (distance per revolution) / (pulses per revolution)
    //  = (Pi * D) / ppr
    public static final double kElevatorEncoderDistPerPulse =
        2.0 * Math.PI * kElevatorDrumRadius / 4096;
  }

    public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1; // updated based on 2025 code
  }
  public static class LimelightConstants {
    public static final double maxVisionDistanceMeters = 4.0; // maximum distance to accept vision measurements
  }
}