// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DoNothing;
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    private SendableChooser<Command> autoChooser;
    private SlewRateLimiter xLimiter = new SlewRateLimiter(7);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(7);
    private SlewRateLimiter omegaLimiter = new SlewRateLimiter(Units.degreesToRadians(950));

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    public RobotContainer() {
        autoChooser = new SendableChooser<Command>();
        configureBindings();
        configureAutonomous();
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(xLimiter.calculate(-joystick.getLeftY() * MaxSpeed)) // Drive
                                                                                                                       // forward
                                                                                                                       // with
                        // negative Y
                        // (forward)
                        .withVelocityY(yLimiter.calculate(-joystick.getLeftX() * MaxSpeed)) // Drive left with negative
                                                                                            // X (left)
                        .withRotationalRate(omegaLimiter.calculate(-joystick.getRightX() * MaxAngularRate)) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                // negative X (left)
                ));

                joystick.y().onTrue(new DriveToPose(() -> drivetrain.getState().Pose));
                // joystick.y().onTrue(drivetrain.pathFindToGoalPose(() -> new Pose2d(5,6,new Rotation2d(90))));
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(
        //         () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public void configureAutonomous() {
        autoChooser.addOption("followLine", drivetrain.executeAuto("StraightLineAuto", false));
        autoChooser.addOption("followLinePath", drivetrain.followPath("StraightLineTwo"));
        autoChooser.addOption("followLineAndRotate", drivetrain.executeAuto("FollowLineAndRotate", false));
        autoChooser.addOption("threeside", drivetrain.executeAuto("ThreeSide", false));
        SmartDashboard.putData(autoChooser);
    }
 


    public Command getAutonomousCommand() {
        Command auto = autoChooser.getSelected();

        if(auto == null) {
            return new DoNothing();
        } else {
            return auto;
        }
    }
}
