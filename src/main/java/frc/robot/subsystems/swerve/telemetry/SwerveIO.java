package frc.robot.subsystems.swerve.telemetry;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveIO {
    @AutoLog
    public static class SwerveIOInputs {
        public double speed = 0.0;
        public double turnSpeed = 0.0;
        public double heading = 0.0;
    }

    public void updateInputs(SwerveIOInputs inputs);
    public void setDesiredModuleStates(SwerveModuleState[] states);
    public void driveRobotRelative(ChassisSpeeds speeds);
    public void driveFieldRelative(ChassisSpeeds speeds);
}
