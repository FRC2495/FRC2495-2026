package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    public SwerveDriveOdometry m_simOdometry = null;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
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
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020)) // https://www.chiefdelphi.com/t/looking-for-an-explanation-of-chassisspeeds-discretize/462069
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
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
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);

        if(m_simOdometry != null) {
            m_simOdometry.resetPose(pose);
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        if (m_simOdometry == null) {
            SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = getModules();
            SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
            for(int i = 0; i < modules.length; ++i) {
                positions[i] = modules[i].getCachedPosition();
            }
            m_simOdometry = new SwerveDriveOdometry(getKinematics(), Rotation2d.kZero, positions);
        }

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
            
            SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = getModules();
            SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
            for(int i = 0; i < modules.length; ++i) {
                positions[i] = modules[i].getCachedPosition();
            }
            m_simOdometry.update(Rotation2d.fromDegrees(getPigeon2().getYaw().getValue().in(Degrees)), positions);
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
    
     /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The timestamp of the pose in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }   
	
	/// begin custom code 
    
    /**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return getState().Pose;
	}


    /**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 * @param rateLimit     Whether to enable rate limiting for smoother control. UNUSED
	 */
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

        if (fieldRelative) {
            driveFieldCentric(xSpeed,ySpeed,rot);
        } else {
            driveRobotCentric(xSpeed,ySpeed,rot);
        }
    }

	public void drive(double xSpeed, double ySpeed, double angularSpeed) {
		this.drive(xSpeed, ySpeed, angularSpeed, true, false);
	}

	public void driveRobotRelative(ChassisSpeeds speeds){
		this.drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,false,true);
	}

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private SwerveRequest.RobotCentric reqRobotCentric= new SwerveRequest.RobotCentric();

    public void driveRobotCentric(double xSpeed, double ySpeed, double rot) {
        reqRobotCentric.VelocityX = xSpeed * MaxSpeed;
        reqRobotCentric.VelocityY = ySpeed * MaxSpeed;
        reqRobotCentric.RotationalRate = rot * MaxAngularRate;
        reqRobotCentric.DriveRequestType = DriveRequestType.OpenLoopVoltage;
        
        super.setControl(reqRobotCentric);
    }

    private SwerveRequest.FieldCentric reqFieldCentric = new SwerveRequest.FieldCentric();

    public void driveFieldCentric(double xSpeed, double ySpeed, double rot) {
        reqFieldCentric.Deadband = MaxSpeed * 0.1;
        reqFieldCentric.RotationalDeadband = MaxAngularRate * 0.1;
        reqFieldCentric.VelocityX = xSpeed * MaxSpeed;
        reqFieldCentric.VelocityY = ySpeed * MaxSpeed;
        reqFieldCentric.RotationalRate = rot * MaxAngularRate;
        reqFieldCentric.DriveRequestType = DriveRequestType.OpenLoopVoltage;
        
        super.setControl(reqFieldCentric);
    }


    /**
	 * Sets the wheels into an X formation to prevent movement.
	 */
	public void setX() {
        brake();
	}

    private final SwerveRequest.SwerveDriveBrake reqBrake = new SwerveRequest.SwerveDriveBrake();

    public void brake() {        
        super.setControl(reqBrake);
    }

    /**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
        // TODO
	}

    public void stop() {
        drive(0, 0, 0, false, false);
    }


    /** Zeroes the heading of the robot. */
	public void zeroHeading() {
		//getPigeon2().reset(); // resets the gyro to a heading of zero
        //tareEverything();
		seedFieldCentric(); // resets the rotation of the robot pose to zero, regardless of gyro heading
	}

    public void oppositeHeading() {
		//getPigeon2().reset(); // resets the gyro to a heading of 180
        //tareEverything();

		Optional<Alliance> alliance = DriverStation.getAlliance();
		Alliance allianceColor = alliance.isPresent() ? alliance.get() : Alliance.Blue;

		if (allianceColor == Alliance.Blue)
		{
			resetRotation(new Rotation2d(Rotation2d.fromDegrees(180).getRadians()));
		}
		else
		{
			resetRotation(new Rotation2d(Rotation2d.fromDegrees(0).getRadians()));
		}
	}

	/*public void blueLeftSubHeading() {
		//getPigeon2().reset(); // resets the gyro to a heading of zero
        //tareEverything();
		resetRotation(new Rotation2d(Rotation2d.fromDegrees(300).getRadians()));
	}*/

    public void startingPositionOneHeading() {
		//getPigeon2().reset(); // resets the gyro to a heading of zero
        //tareEverything();
		resetRotation(new Rotation2d(Rotation2d.fromDegrees(240).getRadians()));
	}

	public void startingPositionTwoHeading() {
		//getPigeon2().reset(); // resets the gyro to a heading of zero
        //tareEverything();
		resetRotation(new Rotation2d(Rotation2d.fromDegrees(120).getRadians()));
	}

	/*public void redLeftSubHeading() {
		getPigeon2().reset(); // resets the gyro to a heading of zero
        tareEverything();
		resetRotation(new Rotation2d(Rotation2d.fromDegrees(300).getRadians()));
	}

	public void redRightSubHeading() {
		getPigeon2().reset(); // resets the gyro to a heading of zero
        tareEverything();
		resetRotation(new Rotation2d(Rotation2d.fromDegrees(60).getRadians()));
	}*/

    public double getHeading() {
        //return getPigeon2().getYaw().getValueAsDouble();
		return getPose().getRotation().getDegrees();
    }


    /** Resets the drive encoders to currently read a position of 0 and seeds the turn encoders using the absolute encoders. */
	public void resetEncoders() {
        getModule(0).resetPosition();
        getModule(1).resetPosition();
        getModule(2).resetPosition();
        getModule(3).resetPosition();
	}

    public double getFrontLeftAbsoluteEncoderPosition() {
        return getModule(0).getEncoder().getAbsolutePosition().getValueAsDouble();
    }

    public double getFrontRightAbsoluteEncoderPosition() {
        return getModule(1).getEncoder().getAbsolutePosition().getValueAsDouble();
    }

    public double getRearLeftAbsoluteEncoderPosition() {
        return getModule(2).getEncoder().getAbsolutePosition().getValueAsDouble();
    }

    public double getRearRightAbsoluteEncoderPosition() {
        return getModule(3).getEncoder().getAbsolutePosition().getValueAsDouble();
    }

    public double getFrontLeftRelativeEncoderPosition() {
        return getModule(0).getSteerMotor().getPosition().getValueAsDouble();
    }

    public double getFrontRightRelativeEncoderPosition() {
        return getModule(1).getSteerMotor().getPosition().getValueAsDouble();
    }

    public double getRearLeftRelativeEncoderPosition() {
        return getModule(2).getSteerMotor().getPosition().getValueAsDouble();
    }
    
    public double getRearRightRelativeEncoderPosition() {
        return getModule(3).getSteerMotor().getPosition().getValueAsDouble();
    }
    
    /**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {

        // TODO
	}
}
