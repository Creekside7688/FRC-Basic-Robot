// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DrivetrainConstants.*;

public class Drivetrain extends SubsystemBase {
    private final WPI_TalonSRX TLFmotor;
    private final WPI_VictorSPX TLBmotor;
    private final WPI_VictorSPX BLFmotor;
    private final WPI_VictorSPX BLBmotor;

    private final WPI_TalonSRX TRFmotor;
    private final WPI_VictorSPX TRBmotor;
    private final WPI_VictorSPX BRBmotor;
    private final WPI_VictorSPX BRFmotor;

    private final MotorControllerGroup leftMotors;
    private final MotorControllerGroup rightMotors;

    private final DifferentialDrive differentialDrive;
    private final DifferentialDriveOdometry odometry;

    private final AHRS gyro;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    public Drivetrain() {
        TLFmotor = new WPI_TalonSRX(TLF_MOTOR);
        TLBmotor = new WPI_VictorSPX(TLB_MOTOR);
        BLFmotor = new WPI_VictorSPX(BLF_MOTOR);
        BLBmotor = new WPI_VictorSPX(BLB_MOTOR);
        TLFmotor.setNeutralMode(NeutralMode.Coast);
        TLBmotor.setNeutralMode(NeutralMode.Coast);
        BLFmotor.setNeutralMode(NeutralMode.Coast);
        BLBmotor.setNeutralMode(NeutralMode.Coast);

        TRFmotor = new WPI_TalonSRX(TRF_MOTOR);
        TRBmotor = new WPI_VictorSPX(TRB_MOTOR);
        BRFmotor = new WPI_VictorSPX(BRF_MOTOR);
        BRBmotor = new WPI_VictorSPX(BRB_MOTOR);
        TRFmotor.setNeutralMode(NeutralMode.Coast);
        TRBmotor.setNeutralMode(NeutralMode.Coast);
        BRFmotor.setNeutralMode(NeutralMode.Coast);
        BRBmotor.setNeutralMode(NeutralMode.Coast);

        leftMotors = new MotorControllerGroup(TLFmotor, TLBmotor, BLFmotor, BLBmotor);
        rightMotors = new MotorControllerGroup(TRFmotor, TRBmotor, BRBmotor, BRFmotor);

        leftMotors.setInverted(false);
        rightMotors.setInverted(true);

        differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

        leftEncoder = new Encoder(LEFT_ENCODER[0], LEFT_ENCODER[1], false);
        rightEncoder = new Encoder(RIGHT_ENCODER[0], RIGHT_ENCODER[1], true);

        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        gyro = new AHRS(Port.kUSB1);

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return -gyro.getRate();
    }

    public void resetOdometry(Pose2d pose) {
        this.resetEncoders();
        odometry.resetPosition(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void resetHeading() {
        gyro.reset();
    }

    public void arcadeDrive(double xSpeed, double tSpeed) {
        differentialDrive.arcadeDrive(xSpeed, tSpeed, false);
    }

    public void tankDrive(double lSpeed, double rSpeed) {
        differentialDrive.tankDrive(lSpeed, rSpeed);
    }

    public void tankDriveVolts(double lVolts, double rVolts) {
        leftMotors.setVoltage(lVolts);
        rightMotors.setVoltage(rVolts);

        differentialDrive.feed();
    }
}
