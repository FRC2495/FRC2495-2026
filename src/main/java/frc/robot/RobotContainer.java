// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.sensors.*;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Hanger;
import frc.robot.vision.LoggableRobotPose;
import frc.robot.vision.PhotonVisionSystem;
import frc.robot.commands.roller.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.hanger.*;
import frc.robot.interfaces.ICamera;
import frc.robot.commands.groups.*;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	public static final double GAMEPAD_AXIS_THRESHOLD = 0.15;
	public static final double JOYSTICK_AXIS_THRESHOLD = 0.15;

	public static final int LX = 0;
	public static final int LY = 1;
	public static final int LT = 2;
	public static final int RT = 3;
	public static final int RX = 4;
	public static final int RY = 5;

	Command indicatorTimedScrollRainbow; // command to run while stating up and when disabled

	boolean isVisionCorrectionEnabled = true; // vision correction is by default enabled 

	// choosers (for auton)

	private final SendableChooser<Command> autoChooser;


	// sensors

	private final HMAccelerometer accelerometer = new HMAccelerometer();

	// drivetrain constants

	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle targetHub = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(10, 0, 0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    private final Telemetry logger = new Telemetry(MaxSpeed);


	// motorized devices

	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final PhotonVisionSystem vision = new PhotonVisionSystem(this::consumePhotonVisionMeasurement, () -> drivetrain.getState().Pose);

	private final WPI_TalonSRX roller_master = new WPI_TalonSRX(Ports.CAN.ROLLER_MASTER);

	private final /*I*/Roller roller = new Roller(roller_master);

	private final TalonFX hanger_master = new TalonFX(Ports.CAN.HANGER_MASTER);
	private final Hanger hanger = new Hanger(hanger_master);

	// pneumatic devices

	//private final Compressor compressor = new Compressor();

	//private final Mouth mouth = new Mouth();

	// misc

	private final Field2d field = new Field2d(); //  a representation of the field

	//private final Indicator indicator = new Indicator(apriltag_camera, object_detection_camera);

	// The driver's and copilot's joystick(s) and controller(s)
	CommandJoystick joyMain = new CommandJoystick(Ports.USB.MAIN_JOYSTICK);
	//CommandXboxController driverGamepad = new CommandXboxController(Ports.USB.DRIVER_GAMEPAD);
	CommandXboxController copilotGamepad = new CommandXboxController(Ports.USB.COPILOT_GAMEPAD);
	CommandJoystick buttonBox = new CommandJoystick(Ports.USB.BUTTON_BOX);
	
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		//autonChooser.setDefaultOption("SP2 One Fuel Test", AUTON_CUSTOM);
		//SmartDashboard.putData("Auto choices", autonChooser); 

		NamedCommands.registerCommand("RollerTimedRollIn", new RollerTimedRollIn(roller, .4));
        NamedCommands.registerCommand("RollerTimedRollOut", new RollerTimedRollOut(roller, .4));
		NamedCommands.registerCommand("RollerStop", new RollerStop(roller));
		NamedCommands.registerCommand("RollerForAutoRollOut", new RollerForAutoRollOut(roller));
		NamedCommands.registerCommand("waitCommand2s", new WaitCommand(2));
		NamedCommands.registerCommand("waitCommand1.5s", new WaitCommand(1.5));
		NamedCommands.registerCommand("waitCommand1s", new WaitCommand(1));

		NamedCommands.registerCommand("Stop Shooting", new WaitCommand(1));
        /* Shoot commands need a bit of time to spool up the flywheel before feeding with the intake */
        NamedCommands.registerCommand("Shoot Near", new WaitCommand(1));
        NamedCommands.registerCommand("Shoot Far", new WaitCommand(1));

        NamedCommands.registerCommand("Stop Intake", new WaitCommand(1));
        NamedCommands.registerCommand("Intake Fuel", new WaitCommand(1));
        NamedCommands.registerCommand("Outtake Fuel", new WaitCommand(1));


		// choosers (for auton)
		
		autoChooser = AutoBuilder.buildAutoChooser("Only Score");
		SmartDashboard.putData("Auto Chooser", autoChooser);

		// Configure the button bindings

		configureButtonBindings();


		// Configure default commands

		drivetrain.setDefaultCommand(
			// The left stick controls translation of the robot.
			// Turning is controlled by the X axis of the right stick.
			// We are inverting LeftY because Xbox controllers return negative values when we push forward.
			// We are inverting LeftX because we want a positive value when we pull to the left. Xbox controllers return positive values when you pull to the right by default.
			// We are also inverting RightX because we want a positive value when we pull to the left (CCW is positive in mathematics).
			// Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joyMain.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joyMain.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joyMain.getZ() * MaxAngularRate) // Drive counterclockwise with negative Z (left)
            ));

		roller.setDefaultCommand(new RollerStopForever(roller)); // we stop by default
		hanger.setDefaultCommand(new HangerStop(hanger));

		//shooter.setDefaultCommand(new ShooterStopForever(shooter)); // we stop by default

		//compressor.checkCompressor(); //we compress in the background

		//indicator.setDefaultCommand(new IndicatorIndicateUsingCamera(indicator)); // default command, only runs when robot is enabled

		//indicatorTimedScrollRainbow = new IndicatorTimedScrollRainbow(indicator,1);
		//indicatorTimedScrollRainbow.schedule(); // we schedule the command as we are starting up

		// Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

		drivetrain.registerTelemetry(logger::telemeterize);

		Trigger hasFuel = new Trigger(() -> roller.hasFuel());
		Trigger noFuelPresent = new Trigger(() -> roller.noFuelPresent() && !roller.isReleasing());
		Trigger isFuelEntering = new Trigger(() -> roller.isFuelEntering() && !roller.isReleasing());
		Trigger isFuelExiting = new Trigger(() -> roller.isFuelExiting() && !roller.isReleasing() && DriverStation.isTeleop());
		//Trigger isFuelReadyToScore = new Trigger(() -> (apriltag_camera.isAtLeftScoringPosition() || apriltag_camera.isAtRightScoringPosition()) && DriverStation.isAutonomous());

		isFuelEntering.whileTrue(
			new RollerRollOutLowRpm(roller)
		);

		isFuelExiting.whileTrue(
			new RollerRollInLowRpm(roller)
		);

		(hasFuel).or(noFuelPresent).whileTrue(
			new RollerStop(roller)
		);

		/*isFuelReadyToScore.whileTrue(
			new RollerForAutoRollOut(roller)
		);*/

		// Warmup PathPlanner to avoid Java pauses
        //CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand()); // we already do that in Robot.RobotInit()
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {

		// driver (joystick)
		
		// Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joyMain.button(1).and(joyMain.button(11)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joyMain.button(1).and(joyMain.button(12)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joyMain.button(2).and(joyMain.button(11)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joyMain.button(2).and(joyMain.button(12)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		// reset the field-centric heading on left bumper press
		joyMain.povUp()
			//.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); 
			.onTrue(new DrivetrainZeroHeading(drivetrain));

		joyMain.povDown()
			.onTrue(new DrivetrainOppositeHeading(drivetrain));

		joyMain.povLeft();
			//.onTrue(new DrivetrainLeftSubHeading(drivetrain));	

		joyMain.povRight();
			//.onTrue(new DrivetrainRightSubHeading(drivetrain));

		joyMain.button(1)
			.whileTrue(drivetrain.applyRequest(() -> brake));

		joyMain.button(2)
			//.whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joyMain.getY(), -joyMain.getX()))));
			.whileTrue(drivetrain.applyRequest(()-> {
				if (!vision.isHubTargetValid()) {
					/* Do typical field-centric driving since we don't have a target */
					return drive.withVelocityX(-joyMain.getY() * MaxSpeed) // Drive forward with negative Y (forward)
						.withVelocityY(-joyMain.getX() * MaxSpeed) // Drive left with negative X (left)
						.withRotationalRate(-joyMain.getZ() * MaxAngularRate); // Drive counterclockwise with negative Z (left)
				} else {
					/* Use the hub target to determine where to aim */
					return targetHub.withTargetDirection(vision.getHeadingToHubFieldRelative())
						.withVelocityX(-joyMain.getY() * MaxSpeed) // Drive forward with negative Y (forward)
						.withVelocityY(-joyMain.getX() * MaxSpeed); // Drive left with negative X (left)
				}
			}  
        	)); // end button 2 binding
			
		joyMain.button(3)
			.onTrue(new EnableVisionCorrection(this, true));		
			
		joyMain.button(4)
			.onTrue(new EnableVisionCorrection(this, false));

        joyMain.button(5);

		joyMain.button(6);

		joyMain.button(7);
			//.whileTrue(new RollerJoystickControl(roller, drivetrain, getMainJoystick()));
			//.whileTrue(new SliderJoystickControl(slider, drivetrain, getMainJoystick()));
		
		joyMain.button(8);
			//.whileTrue(new AlgaeRollerJoystickControl(algae_roller, drivetrain, getMainJoystick()));
		
		joyMain.button(9);
			//.whileTrue(new NeckJoystickControl(neck, drivetrain, getMainJoystick()));
		
		joyMain.button(10);
			//.whileTrue(new ElevatorJoystickControl(elevator, drivetrain, getMainJoystick()));

		joyMain.button(11)
			.whileTrue(new HangerJoystickControl(hanger, drivetrain, getMainJoystick()));
		
		joyMain.button(12)
			.whileTrue(new DrivetrainSetXFormation(drivetrain));
			
				
		// copilot (gamepad)
		
		copilotGamepad.a();
			//.onTrue(new ElevatorMoveToAlgaeLevelTwoWithStallDetection(elevator));
		
		copilotGamepad.b();
			//.onTrue(new ElevatorMoveDownWithStallDetection(elevator));

		copilotGamepad.x();
			//.whileTrue(new RollerRollIn(roller));

		copilotGamepad.y();
			//.whileTrue(new RollerRollOut(roller));
			
		copilotGamepad.back();
			//.onTrue(new DrivetrainAndGyroReset(drivetrain)); TODO
			//.onTrue(new AlmostEverythingStop(elevator, roller, algae_roller));

		copilotGamepad.start()
			.onTrue(new AlmostEverythingStop(roller));


		copilotGamepad.leftTrigger()
			.whileTrue(new RollerRollOut(roller));

		copilotGamepad.rightTrigger();
			//.whileTrue(new AlgaeRollerRelease(algae_roller));
			//.whileTrue(new AlgaeRollerRoll(algae_roller));


		copilotGamepad.povDown();
			//.onTrue(new NeckMoveDownWithStallDetection(neck));
			//.onTrue(new NeckMoveHomeWithStallDetection(neck));

		copilotGamepad.povLeft();
			//.onTrue(new NeckMoveToFuelStationWithStallDetection(neck));

		copilotGamepad.povRight();
			//.onTrue(new NeckMoveToFuelReefWithStallDetection(neck));
			//.onTrue(new NeckMoveToAlgaeReefWithStallDetection(neck));
			//.onTrue(new NeckMoveUpWithStallDetection(neck));

		copilotGamepad.povUp();
			//.onTrue(new NeckMoveUpWithStallDetection(neck));
			//.onTrue(new ElevatorMoveUpWithStallDetection(elevator));
			//.onTrue(new NeckMoveUpWithStallDetection(neck));
			//.onTrue(new NeckMoveToAlgaeReefWithStallDetection(neck));
			//.onTrue(new NeckMoveUpWithStallDetection(neck));


		copilotGamepad.leftBumper();
			//.onTrue(new SliderSafeExtendWithStallDetection(slider, elevator)); // TODO see if safe mode is needed?
			//.onTrue(new ElevatorMoveToFirstLevelWithStallDetection(elevator));

		copilotGamepad.rightBumper();
			//.onTrue(new SliderRetractWithLimitSwitch(slider));


		copilotGamepad.leftStick();
			//.onTrue(new RollerTimedRoll(roller, 3));
			//.onTrue(new GamepadRumble(getCopilotGamepad(),false));
			//.onTrue(new NeckMoveToFuelReefWithStallDetection(neck));

		copilotGamepad.rightStick();
			//.onTrue(new RollerTimedRelease(roller, 3));
			//.onTrue(new GamepadRumble(getCopilotGamepad(),false));
			//.onTrue(new ElevatorMoveUpWithStallDetection(elevator));


		copilotGamepad.axisGreaterThan(LY,GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new HangerGamepadControl(hanger, getCopilotGamepad()));
			//.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));

		copilotGamepad.axisLessThan(LY,-GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new HangerGamepadControl(hanger, getCopilotGamepad()));
			//.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));

		copilotGamepad.axisGreaterThan(LX,GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));
			//.onTrue(new ElevatorMoveToThirdLevelWithStallDetection(elevator));

		copilotGamepad.axisLessThan(LX,-GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));
			//.onTrue(new ElevatorMoveToSecondLevelWithStallDetection(elevator));

		copilotGamepad.axisGreaterThan(RY,GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new ElevatorGamepadControl(elevator, getCopilotGamepad()));

		copilotGamepad.axisLessThan(RY,-GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new ElevatorGamepadControl(elevator, getCopilotGamepad()));
			//.onTrue(new ElevatorMoveToFourthLevelWithStallDetection(elevator));

		copilotGamepad.axisGreaterThan(RX,GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new SliderGamepadControl(slider, getCopilotGamepad()));
			//.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));

		copilotGamepad.axisLessThan(RX,-GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new SliderGamepadControl(slider, getCopilotGamepad()));
			//.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));
		
		// button box 

		buttonBox.button(1);
			//.onTrue(new ElevatorMoveToFirstLevelWithStallDetection(elevator)); 

		buttonBox.button(2);
			//.onTrue(new ElevatorMoveToSecondLevelWithStallDetection(elevator));
		
		buttonBox.button(3);
			//.onTrue(new ElevatorMoveToThirdLevelWithStallDetection(elevator));

		buttonBox.button(4);
			//.onTrue(new ElevatorMoveToFourthLevelWithStallDetection(elevator));
		
		buttonBox.button(5);
			//.onTrue(new ElevatorMoveUpWithStallDetection(elevator));
		
		buttonBox.button(6);
			//.onTrue(new ElevatorMoveDownWithStallDetection(elevator));
			//.onTrue(new ElevatorMoveDownWithStallDetection(elevator));
		
		buttonBox.button(9);
			//.onTrue(new ElevatorMoveToAlgaeLevelTwoWithStallDetection(elevator));

		buttonBox.button(10);
			//.onTrue(new ElevatorMoveToAlgaeLevelThreeWithStallDetection(elevator));
		
		buttonBox.button(11);
			//.whileTrue(new HangerButtonBoxUpControl(hanger));
		
		buttonBox.button(12);
			//.whileTrue(new HangerButtonBoxDownControl(hanger));
		
	}

	

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {

		return autoChooser.getSelected();

		/*autonSelected = autonChooser.getSelected();
		System.out.println("Auton selected: " + autonSelected);	

		switch (autonSelected) {
			case AUTON_CUSTOM:
				return new StartingPositionTwoOneFuel(this, drivetrain, roller, neck, elevator, slider);
				//break;

			case AUTON_DO_NOTHING:
				return null;
				//break;
					
			default:
				// nothing
				return null;
				//break;
			} // end switch*/
		}

	public TrajectoryConfig createFastTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.HIGH_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public TrajectoryConfig createTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.MAX_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public TrajectoryConfig createAlmostMaxTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.MAX_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public TrajectoryConfig createSlowTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.REDUCED_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}


	public TrajectoryConfig createReverseTrajectoryConfig() {

		TrajectoryConfig config = createTrajectoryConfig();

		config.setReversed(true); // in reverse!

		return config;
	}

	public TrajectoryConfig createFastReverseTrajectoryConfig() {

		TrajectoryConfig config = createFastTrajectoryConfig();

		config.setReversed(true); // in reverse!

		return config;
	}

	/*public Trajectory createExampleTrajectory() {
		// An example trajectory to follow. All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
			createTrajectoryConfig());

		return exampleTrajectory;
	}*/
	
	/*public Command createSwerveControllerCommand(Trajectory trajectory) {

		ProfiledPIDController thetaController = new ProfiledPIDController(
			AutoConstants.THETA_CONTROLLER_P, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
			
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
			trajectory, // trajectory to follow
			drivetrain::getPose, // Functional interface to feed supplier
			DrivetrainConstants.DRIVE_KINEMATICS, // kinematics of the drivetrain
			new PIDController(AutoConstants.X_CONTROLLER_P, 0, 0), // trajectory tracker PID controller for x position
			new PIDController(AutoConstants.Y_CONTROLLER_P, 0, 0), // trajectory tracker PID controller for y position
			thetaController, // trajectory tracker PID controller for rotation
			drivetrain::setModuleStates, // raw output module states from the position controllers
			drivetrain); // subsystems to require

		// Reset odometry to the starting pose of the trajectory.
		drivetrain.resetOdometry(trajectory.getInitialPose()); // WARNING: https://github.com/REVrobotics/MAXSwerve-Java-Template/issues/13

		field.getObject("trajectory").setTrajectory(trajectory);

		// Run path following command, then stop at the end.
		return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false, false));
	}*/


	public Field2d getField()
	{
		return field;
	}

	public HMAccelerometer getAccelerometer()
	{
		return accelerometer;
	}


	public CommandSwerveDrivetrain getDrivetrain()
	{
		return drivetrain;
	}

	public Hanger getHanger()
	{
		return hanger;
	}


	public Roller getRoller()
	{
		return roller;
	}

	/*public Shooter getShooter()
	{
		return shooter;
	}*/

	public String getAllianceColor() 
	{
		Optional<Alliance> alliance = DriverStation.getAlliance();
		Alliance allianceColor = alliance.isPresent() ? alliance.get() : Alliance.Blue;
		return allianceColor.toString();
	}

	public Joystick getMainJoystick()
	{
		return joyMain.getHID();
	}

	public XboxController getCopilotGamepad()
	{
		return copilotGamepad.getHID();
	}

	public Joystick getButtonBox() 
	{
		return buttonBox.getHID();
	}

	/*public SendableChooser<String> getAutonChooser()
	{
		return autonChooser;
	}*/

	public boolean getVisionCorrectionEnablement() 
	{
		return isVisionCorrectionEnabled;
	}
	
	public void changeVisionCorrectionEnablement(boolean visionUse) 
	{
		isVisionCorrectionEnabled = visionUse;
	}

	public void consumePhotonVisionMeasurement(LoggableRobotPose pose) {
        
		if (getVisionCorrectionEnablement() == false) {
			return; // do not consume measurements if vision correction is disabled
		}

		/* Super simple, should modify to support variable standard deviations */
        drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }

    public void periodic() {
        vision.periodic();
    }

    public void simulationPeriodic() {
        var drivetrainPose = drivetrain.m_simOdometry.getPoseMeters();
        vision.simPeriodic(drivetrainPose);

        var debugField = vision.getSimDebugField();
        debugField.getObject("EstimatedRobot").setPose(drivetrainPose);
    }
}
