// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static class AprilTags {

		// facing the field elements (or from the driver station for the stages)
		public static final int RED_LEFT_FUEL_STATION = 1; 
		public static final int RED_RIGHT_FUEL_STATION = 2;
		public static final int BLUE_PROCESSOR = 3;
		public static final int RED_ALLIANCE_BLUE_NET = 4; 
		public static final int RED_ALLIANCE_RED_NET = 5; 
		public static final int RED_REEF_SIDE_D = 6; // target
		public static final int RED_REEF_SIDE_E = 7; // target
		public static final int RED_REEF_SIDE_F = 8; // target
		public static final int RED_REEF_SIDE_A = 9; // target
		public static final int RED_REEF_SIDE_B = 10; // target
		public static final int RED_REEF_SIDE_C = 11; // target
		public static final int BLUE_RIGHT_FUEL_STATION = 12;
		public static final int BLUE_LEFT_FUEL_STATION = 13;
		public static final int BLUE_ALLIANCE_BLUE_NET = 14; 
		public static final int BLUE_ALLIANCE_RED_NET = 15;
		public static final int RED_PROCESSOR = 16;
		public static final int BLUE_REEF_SIDE_D = 17; // target
		public static final int BLUE_REEF_SIDE_E = 18; // target
		public static final int BLUE_REEF_SIDE_F = 19; // target
		public static final int BLUE_REEF_SIDE_A = 20; // target
		public static final int BLUE_REEF_SIDE_B = 21; // target
		public static final int BLUE_REEF_SIDE_C = 22; // target
	
	}


	public static final class DrivetrainConstants {
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		public static final double MAX_SPEED_METERS_PER_SECOND = 4.6; //4.42; //4.8;
		public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI; // radians per second

		public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
		public static final double MAGNITUDE_SLEW_RATE = 1.8; // 2.0; //1.8; // percent per second (1 = 100%)
		public static final double ROTATIONAL_SLEW_RATE = 2.0; // 20.0; //2.0; // percent per second (1 = 100%)

		// Chassis configuration
		public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(24);
		
		// Distance between centers of right and left wheels on robot
		public static final double WHEEL_BASE_METERS = Units.inchesToMeters(24);
		
		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
				new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
				new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
				new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
				new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));

		public static final boolean kGyroReversed = false;
	}

	public static final class SwerveModuleConstants {
		// The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
		// This changes the drive speed of the  (a pinion gear with more teeth will result in a
		// robot that drives faster).
		public static final int kDrivingMotorPinionTeeth = 14;

		// Invert the turning encoder, since the output shaft rotates in the opposite direction of
		// the steering motor in the MAXSwerve Module.
		public static final boolean kTurningEncoderInverted = true;

		// Calculations required for driving motor conversion factors and feed forward
		public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
		public static final double WHEEL_DIAMETER_METERS = 0.1016;
		public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
		public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 17 * 50) / (kDrivingMotorPinionTeeth * 15 * 27);
		public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)	/ DRIVING_MOTOR_REDUCTION;

		public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION = (WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION; // meters, per rotation
		public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM = ((WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second, per RPM

		public static final double TURNING_MOTOR_REDUCTION = 150.0 / 7.0; // ratio between internal relative encoder and Through Bore (or Thrifty in our case) absolute encoder - 150.0 / 7.0

		public static final double TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION = (2 * Math.PI) / TURNING_MOTOR_REDUCTION ; // radians, per rotation
		public static final double TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM = (2 * Math.PI) / TURNING_MOTOR_REDUCTION / 60.0; // radians per second, per RPM

		public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
		public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS = (2 * Math.PI); // radians

		public static final double DRIVING_P = 0.04;
		public static final double DRIVING_I = 0;
		public static final double DRIVING_D = 0;
		public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
		public static final double DRIVING_MIN_OUTPUT_NORMALIZED = -1;
		public static final double DRIVING_MAX_OUTPUT_NORMALIZED = 1;

		public static final double TURNING_P = 1.0; // 1.0 might be a bit too much - reduce a bit if needed
		public static final double TURNING_I = 0;
		public static final double TURNING_D = 0;
		public static final double TURNING_FF = 0;
		public static final double TURNING_MIN_OUTPUT_NORMALIZED = -1;
		public static final double TURNING_MAX_OUTPUT_NORMALIZED = 1;

		public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
		public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

		public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 40; //50; // amps
		public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20; // amps
	}

	public static final class AutoConstants {
		public static final double MAX_SPEED_METERS_PER_SECOND = 4.0; //2.85;//4.0; //4.42; //3.0;
		public static final double ALMOST_MAX_SPEED_METERS_PER_SECOND = 3.0; //2.85;//4.0; //4.42; //3.0;
		public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
		public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
		public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

		public static final double HIGH_SPEED_METERS_PER_SECOND = 5.0;

		public static final double REDUCED_SPEED_METERS_PER_SECOND = 2; //4.42; //3.0;

		public static final double X_CONTROLLER_P = 1;
		public static final double Y_CONTROLLER_P = 1;
		public static final double THETA_CONTROLLER_P = 1;

		public static final double TRANSLATION_HOLONOMIC_CONTROLLER_P = 3;//3.1; //2.95;//2.8;//2.9; //.5; //2.75; //4.0;
		public static final double ROTATION_HOLONOMIC_CONTROLLER_P = 2;//1;//1.8;//2;//1; //2; //5; //2.5

		public static final double TRANSLATION_HOLONOMIC_CONTROLLER_I = 0;
		public static final double ROTATION_HOLONOMIC_CONTROLLER_I = 0;
		
		// Constraint for the motion profiled robot angle controller
		public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
			MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
	}

	public static final class NeoMotorConstants {
		public static final double FREE_SPEED_RPM = 5676;
	}

	public static final class VisionConstants {
		// blue side
		public static final double X_LEFT_ALIGNMENT = 0.35; //meters TODO fix value
		public static final double Y_LEFT_ALIGNMENT = -0.14;//-0.19;
		public static final double X_RIGHT_ALIGNMENT = 0.35;
		public static final double Y_RIGHT_ALIGNMENT = 0.19;//0.14;
		public static final double ROT_ALIGNMENT = Math.PI;//-Math.PI/2; //radians
		public static double MAX_ALIGNMENT_DISTANCE = 1.5;
	
		public static final double X_ALIGNMENT_TOLERANCE = .02;//.05;
		public static final double Y_ALIGNMENT_TOLERANCE = .01;//.05;
		public static final double ROT_ALIGNMENT_TOLERANCE = .05;//.05;

		public static final double X_ALIGNMENT_AUTO_TOLERANCE = .02;//.05;
		public static final double Y_ALIGNMENT_AUTO_TOLERANCE = .02;//.05;
		public static final double ROT_ALIGNMENT_AUTO_TOLERANCE = .05;//.05;

		public static final double X_REEF_ALIGNMENT_P = .4;//.55;
		public static final double Y_REEF_ALIGNMENT_P = .4;//.55;
		public static final double ROT_REEF_ALIGNMENT_P = .25;//.4;//1;//0.058;

		public static final double DONT_SEE_TAG_WAIT_TIME = 0.3;
		public static final double POSE_VALIDATION_TIME = 0.3;
	}
}
