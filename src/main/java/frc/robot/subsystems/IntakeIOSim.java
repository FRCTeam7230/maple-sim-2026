package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim {
    private final IntakeSimulation intakeSimulation;
    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
        // Here, create the intake simulation with respect to the intake on your real robot
        

        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        // Specify the type of game pieces that the intake can collect
        "Fuel",
        // Specify the drivetrain to which this intake is attached
        driveTrain,
        // Width of the intake
        Meters.of(0.7),
        // The extension length of the intake beyond the robot's frame (when activated)
        Meters.of(0.2),
        // The intake is mounted on the back side of the chassis
        IntakeSimulation.IntakeSide.BACK,
        // The intake can hold up to 1 note
        1);
        
        @Override // Defined by IntakeIO
        public void setRunning(boolean runIntake) {
            if (runIntake)
                intakeSimulation.startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with game pieces
            else
                intakeSimulation.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
        }

        @Override // Defined by IntakeIO
        public boolean isNoteInsideIntake() {
            return intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
        }

    @Override // Defined by IntakeIO
    public void launchNote() {
        // if there is a note in the intake, it will be removed and return true; otherwise, returns false
        if (intakeSimulation.obtainGamePieceFromIntake())
            ShooterIOSim.launchNote(); // notify the simulated flywheels to launch a note
        }
    }
}