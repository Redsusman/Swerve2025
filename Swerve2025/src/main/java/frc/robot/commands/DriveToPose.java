// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  // Create PID controllers with constraints
  Supplier<Pose2d> targetPose;
  Supplier<Pose2d> robotPose;
  PIDController xController;// Tune
  PIDController yController;// Tune
  PIDController thetaController; // Tune
  CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
  SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
  // SwerveRequest.FieldCentricFacingAngle facing = new
  // SwerveRequest.FieldCentricFacingAngle();

  public DriveToPose(Supplier<Pose2d> robotPose) {
    this.robotPose = robotPose;
    this.targetPose = () -> LimelightHelpers.getTargetPose3d_CameraSpace("").toPose2d();
    xController = new PIDController(0.1, 0, 0);
    yController = new PIDController(0.1, 0, 0);
    thetaController = new PIDController(0.3, 0.0, 0.02);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Set tolerances for the controllers
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    thetaController.setTolerance(Units.degreesToRadians(5));

    xController.reset();
    yController.reset();
    thetaController.reset();

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {
    Pose2d goalPose = targetPose.get();
    Pose2d currentPose = robotPose.get();
    Pose2d transformedPose = goalPose.transformBy(new Transform2d(Constants.AuxiallryConstants.poseTransforms.get(1),
        goalPose.getRotation().unaryMinus().minus(currentPose.getRotation())));
    // Pose2d backupPose = transformedPose.transformBy(new Transform2d(
    // new Translation2d(0.0, 0.0),
    // transformedPose.getRotation().unaryMinus().minus(currentPose.getRotation())));

    double xSetpoint = xController.calculate(currentPose.getX(), transformedPose.getX());
    double ySetpoint = yController.calculate(currentPose.getY(), transformedPose.getY());
    double thetaSetpoint = thetaController.calculate(currentPose.getRotation().getRadians(),
        transformedPose.getRotation().getRadians());

    swerve.setControl(robotSpeeds.withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xSetpoint, ySetpoint,
        thetaSetpoint, currentPose.getRotation())));

    // System.out.println("xError: " + xController.getError() +" yError: " +
    // yController.getError() + " thetaError: " + thetaController.getError());

  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    swerve.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));

    System.out.println("Drive to Pose Ended" + (interrupted ? " (interrupted)" : ""));
  }

  @Override
  public boolean isFinished() {
    // Check if the controllers have reached their goals
    boolean finished = xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    if (finished) {
      System.out.println("Drive to Pose Finished");
    }
    return finished;
  }
}
