package frc.robot.subsystems.swerve.normal;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveModuleNeo[] modules;

    private ADXRS450_Gyro gyro;

    private PIDController thetaController;

    public SwerveSubsystem() {
        for(int id = 0; id < 4; id++) {
            modules[id] = new SwerveModuleNeo(id);
        }
        gyro = new ADXRS450_Gyro(SPI.Port.kMXP);

        thetaController = new PIDController(DriveConstants.kPTheta, DriveConstants.kITheta, DriveConstants.kDTheta);
        thetaController.enableContinuousInput(0, 360);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        var moduleStates = toModuleStates(speeds);
        setDesiredModuleStates(moduleStates);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        driveRobotRelative(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRotation2d()));
    }

    public void setDesiredModuleStates(SwerveModuleState[] moduleStates) {
        for(int id = 0; id < modules.length; id++) {
            modules[id].setModuleState(moduleStates[id]);
        }
    }

    public SwerveModuleState[] toModuleStates(ChassisSpeeds speeds) {
        return DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    }

    public ChassisSpeeds toChassisSpeeds(SwerveModuleState[] states) {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(states);
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public double getHeading() {
        return gyro.getAngle();
    }
}
