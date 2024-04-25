package frc.robot.subsystems.swerve.normal;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModuleConstants;

public class SwerveModuleNeo {
    private CANSparkMax driveMotor, turningMotor;
    private RelativeEncoder driveEncoder, turningRelativeEncoder;
    private SparkPIDController driveController, turnController;

    private CANcoder turningAbsoluteEncoder;

    public SwerveModuleNeo(int id) {
        if(id < 0 || id > 3) {
            throw new IndexOutOfBoundsException("Swerve ID given is out of bounds of length 4. ID given: "+ id);
        }

        driveMotor = new CANSparkMax(DriveConstants.DRIVE_PORTS[id], MotorType.kBrushless);
        driveMotor.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
        driveMotor.setSecondaryCurrentLimit(DriveConstants.kCurrentLimit);
        driveMotor.setInverted(DriveConstants.DRIVE_REVERSED[id]);

        driveController = driveMotor.getPIDController();
        driveController.setP(ModuleConstants.kPDrive);
        driveController.setI(ModuleConstants.kIDrive);
        driveController.setD(ModuleConstants.kDDrive);
        driveController.setOutputRange(-1, 1);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setInverted(DriveConstants.DRIVE_REVERSED[id]);

        turningMotor = new CANSparkMax(DriveConstants.TURNING_PORTS[id], MotorType.kBrushless);
        turningMotor.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
        turningMotor.setSecondaryCurrentLimit(DriveConstants.kSmartCurrentLimit);
        turningMotor.setInverted(DriveConstants.TURNING_REVERSED[id]);

        turnController = turningMotor.getPIDController();
        turnController.setP(ModuleConstants.kPTurning);
        turnController.setI(ModuleConstants.kITurning);
        turnController.setD(ModuleConstants.kDTurning);
        turnController.setOutputRange(-1, 1);

        turningRelativeEncoder = turningMotor.getEncoder();
        turningRelativeEncoder.setInverted(DriveConstants.TURNING_REVERSED[id]);

        turningAbsoluteEncoder = new CANcoder(DriveConstants.CANCODER_PORTS[id]);

        var magnetConfigs = new MagnetSensorConfigs();
        magnetConfigs.withMagnetOffset(DriveConstants.ENCODER_OFFSETS[id]);
        magnetConfigs.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
        magnetConfigs.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        turningAbsoluteEncoder.getConfigurator().apply(magnetConfigs);
        
    }

    public void setModuleState(SwerveModuleState state) {
        setReferenceAngle(state.angle.getDegrees());
        setDriveVelocity(state.speedMetersPerSecond);
    }

    public void setReferenceAngle(double angle) {
        double currentAngle = getTurnPosition();

        double oppositeAngle = (currentAngle + 180) % 360.0;

        double diff1 = angle - currentAngle;
        double diff2 = oppositeAngle - angle;

        if(Math.abs(diff1) < Math.abs(diff2)) {
            setTurnPosition(currentAngle + diff1);
        } else {
            setTurnPosition(currentAngle + diff2);
        }
    }

    public void setDriveVelocity(double velocityMPS) {
        driveController.setReference(velocityMPS / ModuleConstants.kDriveEncoderRPM2MeterPerSec, ControlType.kVelocity);
    }

    public void setTurnPosition(double degrees) {
        driveController.setReference(degrees / ModuleConstants.kTurningEncoderRot2Deg, ControlType.kPosition);
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            Rotation2d.fromDegrees(getTurnPosition())
        );
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            getDrivePosition(),
            Rotation2d.fromDegrees(getTurnPosition())
        );
    }

    public void updateTurningEncoder() {
        turningRelativeEncoder.setPosition(getAbsolutePosition());
    }

    public boolean isTurningEncoderAligned() {
        return Math.abs(turningRelativeEncoder.getPosition() - turningAbsoluteEncoder.getPosition().getValueAsDouble()) < ModuleConstants.MAX_TURN_ERROR_DEGREES;
    }

    /** Meters */
    private double getDrivePosition() {
        return driveEncoder.getPosition() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    /** Meters per second */
    private double getDriveVelocity() {
        return driveEncoder.getVelocity() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    /** Degrees */
    private double getTurnPosition() {
        return (turningRelativeEncoder.getPosition() % 360) * ModuleConstants.kTurningEncoderRot2Deg;
    }

    /** Degrees per second */
    private double getTurnVelocity() {
        return turningRelativeEncoder.getVelocity() * ModuleConstants.kTurningEncoderRPM2DegPerSec;
    }

    /** Degrees */
    private double getAbsolutePosition() {
        return turningAbsoluteEncoder.getPosition().getValueAsDouble() * 360.0;
    }

    /** Degrees per second */
    private double getAbsoluteVelocity() {
        return turningAbsoluteEncoder.getVelocity().getValueAsDouble() * 360.0 / 60.0;
    }
}