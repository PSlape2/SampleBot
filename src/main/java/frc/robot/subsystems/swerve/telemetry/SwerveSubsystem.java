package frc.robot.subsystems.swerve.telemetry;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveIO io;
    private static final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

    private PIDController thetaController;

    public SwerveSubsystem(SwerveIO io) {
        this.io = io;

        thetaController = new PIDController(DriveConstants.kPTheta, DriveConstants.kITheta, DriveConstants.kDTheta);
    }

    public SwerveSubsystem() {
        this(new SwerveIOReal());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SwerveSubsystem", inputs);
    }

    public void setDesiredModuleStates(SwerveModuleState[] states) {
        io.setDesiredModuleStates(states);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        io.driveRobotRelative(speeds);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        io.driveFieldRelative(speeds);
    }

    public double getSpeed() {
        return inputs.speed;
    }

    public double getHeading() {
        return inputs.heading;
    }
}
