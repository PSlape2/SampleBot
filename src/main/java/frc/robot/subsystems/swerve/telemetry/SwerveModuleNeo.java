package frc.robot.subsystems.swerve.telemetry;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleNeo {
    /** IDs:
     *      Front Left - 1
     *      Back Left - 2
     *      Front Right - 3
     *      Back Right - 4
     */
    private int id;
    private SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public SwerveModuleNeo(int id) {
        this.id = id;
        io = new SwerveModuleIONeo(id);
    }

    public void update() {
        io.updateInputs(inputs);
        Logger.processInputs("SwerveModule" + id, inputs);
    }

    public SwerveModuleNeo(SwerveModuleIO io) {
        this.io = io;
    }

    public void setDesiredModuleState(SwerveModuleState state) {
        io.setDesiredModuleState(state);
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveSpeed(), Rotation2d.fromDegrees(getTurnPosition()));
    }
    
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurnPosition()));
    }

    public double getDrivePosition() {
        return inputs.drivePos;
    }

    public double getDriveSpeed() {
        return inputs.drivePos;
    }

    public double getTurnPosition() {
        return inputs.turnPos;
    }
}
