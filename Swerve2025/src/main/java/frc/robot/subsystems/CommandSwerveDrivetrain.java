package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.wpiutils.AutoFeedEnable;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.DoNothing;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static CommandSwerveDrivetrain instance = null;
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        LimelightHelpers.setCameraPose_RobotSpace("", -0.0762, 0.0, 0.2921, 0.0, 0.0, 0.0);
        try {

            AutoBuilder.configure(
                    () -> getState().Pose,
                    this::resetPose, // confirm if this actually resets pose, original this::resetPose, or explicitly
                                     // call before.
                    () -> getState().Speeds,
                    (speeds, feedforward) -> setControl(robotSpeeds.withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforward.robotRelativeForcesX())
                            .withWheelForceFeedforwardsY(feedforward.robotRelativeForcesY())),
                    new PPHolonomicDriveController(new PIDConstants(6, 0, 0), new PIDConstants(6, 0, 0)), // tune
                    RobotConfig.fromGUISettings(),
                    () -> {
                        if (DriverStation.getAlliance().get() == Alliance.Red) {
                            return false;
                        } else {
                            return true;
                        }
                    }, this);
        } catch (IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        // this.resetPose(new Pose2d(17.069,1.591,new Rotation2d(0.0)));
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */

        // LimelightHelpers.SetRobotOrientation("", this.getPigeon2().getYaw().getValueAsDouble(), 0.0, 0.0, 0.0, 0.0,
        //         0.0);
        // PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed(""); // change depending on alliance
        // Pose2d pose = poseEstimate.pose;
        // double timestamp = poseEstimate.timestampSeconds;

        // addVisionMeasurement(pose, timestamp); // check if should use Utis.getCurrentTimestampSeconds() instead of
                                               // poseEstimate.timestampSeconds

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Command followPath(String pathName) {
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile(pathName);

        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        if (path == null) {
            return Commands.none();
        }
        // FlippingUtil.flipFieldPose(path.getStartingHolonomicPose().get());
        return AutoBuilder.followPath(path);

    }

    public static CommandSwerveDrivetrain getInstance() {
        if (instance == null) {
            instance = TunerConstants.createDrivetrain();
        }
        return instance;
    }

    public Command executeAuto(String autoName, boolean mirror) {
        return new PathPlannerAuto(autoName, mirror);
    }

    public Command driveToGoalTransform(boolean usePose, boolean useTransformedPose, Pose2d possiblePose) { //test with presetPossiblePose
        Command driveCommand = new Command() {
             Pose3d goalPose3d = LimelightHelpers.getTargetPose3d_CameraSpace("");
        Pose2d targetPose = usePose ? possiblePose : goalPose3d.toPose2d();
        Pose2d currentPose = getState().Pose;
        Pose2d transformedTargetPose = currentPose.transformBy(new Transform2d(targetPose.getTranslation(), targetPose.getRotation()));
        Pose2d calculatedPose = useTransformedPose ? transformedTargetPose : targetPose;
        
            ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0,
                    new TrapezoidProfile.Constraints(2.5, 1.0)); // tune
            ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0,
                    new TrapezoidProfile.Constraints(2.5, 1.0)); // tune
            ProfiledPIDController thetaController = new ProfiledPIDController(4, 0, 0,
                    new TrapezoidProfile.Constraints(Units.degreesToRadians(45), 1.0)); // tune

            @Override
            public void initialize() {
                xController.setTolerance(0.1);
                yController.setTolerance(0.1);
                thetaController.setTolerance(Units.degreesToRadians(2));
                xController.reset(currentPose.getX());
                yController.reset(currentPose.getY());
                thetaController.reset(currentPose.getRotation().getRadians());
            }
            @Override
            public void execute() {
                double xVolts = xController.calculate(currentPose.getX(), calculatedPose.getX());
                double yVolts = yController.calculate(currentPose.getY(), calculatedPose.getY());
                double thetaVolts = thetaController.calculate(calculatedPose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians());
                setControl(robotSpeeds.withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xVolts, yVolts, thetaVolts, currentPose.getRotation())));
                // HolonomicDriveController controller;


            }

            @Override
            public void end(boolean interrupted) {
                setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0)));

            }

            @Override
            public boolean isFinished() {
                // return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint(); // omit if
                            
            //   return false;
            return xController.atGoal() && yController.atGoal() && thetaController.atGoal();// omit if you want to run forever
                                                                                                             // to false
                                                                                                             // instead
            }
        };
        double tagNumber = LimelightHelpers.getFiducialID("");
        // if(tagNumber != 9 && tagNumber != 10){
        //     return new DoNothing();
        // } else {
            return driveCommand;
        // }
    }

}
