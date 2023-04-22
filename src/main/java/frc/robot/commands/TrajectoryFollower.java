// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryFollower extends SequentialCommandGroup {
    private final Drivetrain drivetrain;
    private final String trajectoryFile;

    private final DifferentialDriveVoltageConstraint voltageConstraint;
    private Trajectory trajectory;
    private final TrajectoryConfig trajectoryConfig;

    private final RamseteCommand ramseteCommand;

    public TrajectoryFollower(Drivetrain drivetrain, String trajectoryFile) {
        this.drivetrain = drivetrain;
        this.trajectoryFile = trajectoryFile;

        voltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        DrivetrainConstants.KS_VOLTS,
                        DrivetrainConstants.KV_VOLT_SECONDS_PER_METRE,
                        DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METRE),
                DrivetrainConstants.DRIVE_TRAIN_KINEMATICS,
                10);

        trajectoryConfig = new TrajectoryConfig(
                DrivetrainConstants.Auto.MAX_SPEED_METRES_PER_SECOND,
                DrivetrainConstants.Auto.MAX_ACCELERATION_METRES_PER_SECOND_SQUARED)
                        .setKinematics(DrivetrainConstants.DRIVE_TRAIN_KINEMATICS)
                        .addConstraint(voltageConstraint);

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(this.trajectoryFile);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch(IOException e) {
            DriverStation.reportError("Unable to open trajectory file.", e.getStackTrace());
            trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    List.of(new Translation2d(0, 0)),
                    new Pose2d(0, 0, new Rotation2d(0)),
                    trajectoryConfig);
        }

        ramseteCommand = new RamseteCommand(
                trajectory,
                drivetrain::getPose,
                new RamseteController(
                        DrivetrainConstants.RAMSETE_B,
                        DrivetrainConstants.RAMSETE_ZETA),
                new SimpleMotorFeedforward(
                        DrivetrainConstants.KS_VOLTS,
                        DrivetrainConstants.KV_VOLT_SECONDS_PER_METRE,
                        DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METRE),
                DrivetrainConstants.DRIVE_TRAIN_KINEMATICS,
                drivetrain::getWheelSpeeds,
                new PIDController(DrivetrainConstants.KP_DRIVE, 0, 0),
                new PIDController(DrivetrainConstants.KP_DRIVE, 0, 0),
                drivetrain::tankDriveVolts,
                drivetrain);

        addCommands(ramseteCommand.andThen(() -> this.drivetrain.tankDriveVolts(0, 0)));
    }
}
