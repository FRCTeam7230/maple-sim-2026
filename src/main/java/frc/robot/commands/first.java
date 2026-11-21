/* import frc.robot.subsystems.drive.Drive;
import frc.robot.commands.DriveCommands;

public class practice {
    public void practice(JoystickButton m_drivercontroller, Drive drive) {
        m_drivercontroller.a().whileTrue(DriveCommands.robotJoystickDrive(Drive drive,0.1,0.0,0.0));
    }
} */

//excercise 2:
/*public class practice {
    public static void main(String[] args) {
    m_drivercontroller.b().whileTrue(DriveCommands.robotJoystickDrive(Drive drive,0.0,0.0,0.1));
}} */

//excercise 3:
/*public class practice {
    public static void main(String[] args) {
    m_drivercontroller.x().onTrue(DriveCommands.robotJoystickDrive(Drive drive,0.1,0.0,0.1));
    m_drivercontroller.x().onFalse(DriveCommands.robotJoystickDrive(Drive drive,-0.1,0.0,0.0));
}}*/

//excercise 4:

/*public class practice {
    private final double x;
    private final double y;
    public practice(double x, double y) {
        this.x=x;
        this.y=y;
    }
    public double getX() {
        return this.x;
    }
    public double getY() {
        return this.y;
    }
} *

/* public class calculation {
    private calculation() {} //java always need a default constructor
    public static double cal(Drive c) { //a method is always needed for operations. Parameters are objects.
        double a = c.getPose().getX();
        double horizontalLocation = a-8.27;
        if (horizontalLocation>0) {
            m_drivercontroller.leftTrigger().onTrue(DriveCommands.robotJoystickDrive(Drive drive,0.0,-0.1,0.0));
        } else if (horizontalLocation<0) {
            m_drivercontroller.leftTrigger().onFalse(DriveCommands.robotJoystickDrive(Drive drive,0.1,0.0,0.0));
        }
    }
} */

/* public class calculation extends Command{
    public Drive a;
    private calculation(Drive c) { //constructor
        a = c;
    }
    public static double cal(Drive c) { //a method is always needed for operations. Parameters are objects.
        double a = c.getPose().getX();
        double horizontalLocation = a-8.27;
        if (horizontalLocation>0) {
            m_drivercontroller.leftTrigger().onTrue(DriveCommands.robotJoystickDrive(Drive drive,0.0,-0.1,0.0));
        } else if (horizontalLocation<0) {
            m_drivercontroller.leftTrigger().onFalse(DriveCommands.robotJoystickDrive(Drive drive,0.1,0.0,0.0));
        }
    }
    @Override
    public void execute() {
        cal(Drive c)
    }
} */