package frc.robot;

/**
 * Contains the definitions of all the ports
 */
public class Ports {

		// IP (v4) addresses
		// The purpose of this section is to serve as a reminder of what static IP (v4) addresses are used so they are consistent
		// between the competition and practice robots.
		//
		// The radio is automatically set to 10.24.95.1
		// The radio configuration IP is http://192.168.69.1/configuration
		// The Access Point radio WPA is hivemind
		// The Rio is set to static 10.24.95.2, mask 255.255.255.0
		// The Limelight is set to 10.24.95.11, mask 255.255.255.0, gateway 10.24.95.1
		// but note that pressing the reset button will revert to DHCP.
		// The Raspberry Pi running FRCVision is set to static 10.24.95.12, mask 255.255.255.0, gateway 10.24.95.1, DNS blank
		//
		// If a device cannot be accessed (e.g. because its address was somehow obtained via DHCP and mDNS is not working),
		// use Angry IP Scanner to find it!


		/**
		 * Digital ports
		 */
		public static class Digital {
			public static final int CHECK_PRESSURE = 0;
			public static final int FRONT_FUEL_SENSOR = 1;
			public static final int BACK_FUEL_SENSOR = 2;
			//public static final int NOTE_SENSOR = 3;
			//public static final int NOTE_SENSOR_TWO = 2;
		}
		
		/**
		 * Analog ports
		 */
		public static class Analog {
			//public static final int SONAR = 3;
			//public static final int PRESSURE_SENSOR = 1;

			// 2023 Off-season
			// SPARK MAX Absolute encoders
			
			// public static final int FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER = 0;
			// public static final int REAR_RIGHT_TURNING_ABSOLUTE_ENCODER = 1;
			// public static final int REAR_LEFT_TURNING_ABSOLUTE_ENCODER = 2;
			// public static final int FRONT_LEFT_TURNING_ABSOLUTE_ENCODER = 3;			
		}
		
		/**
		 * Relays
		 */
		public static class Relay {
			public static final int COMPRESSOR_RELAY = 0;
		}
		
		/**
		 * CAN Ids
		 */
		public static class CAN {
			//2025 Robot
			public static final int PCM = 1;
			public static final int PDP = 0;	

			// SPARK MAX CAN IDs
			public static final int FRONT_LEFT_DRIVING = 8;
			public static final int REAR_LEFT_DRIVING = 6;
			public static final int FRONT_RIGHT_DRIVING = 2;
			public static final int REAR_RIGHT_DRIVING = 4;

			public static final int FRONT_LEFT_TURNING = 7;
			public static final int REAR_LEFT_TURNING = 5;
			public static final int FRONT_RIGHT_TURNING = 1;
			public static final int REAR_RIGHT_TURNING = 3;

			public static final int NECK_MASTER = 11;

			// TALON SRX CAN IDS
			public static final int ELEVATOR_MASTER = 9;
			public static final int ELEVATOR_FOLLOWER = 10;
			public static final int ROLLER_MASTER = 14;

			public static final int HANGER_MASTER = 16; //TODO fix
			//public static final int SHOOTER_MASTER = 15;
			//public static final int SHOOTER_FOLLOWER = 16;
			//public static final int DRAWER = 17;
			
			// GYRO CAN IDS
			public static final int PIGEON_DRIVETRAIN = 17;

			// CANCODER CAN IDS
			public static final int FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER = 18;
			public static final int REAR_RIGHT_TURNING_ABSOLUTE_ENCODER = 19;
			public static final int REAR_LEFT_TURNING_ABSOLUTE_ENCODER = 20;
			public static final int FRONT_LEFT_TURNING_ABSOLUTE_ENCODER = 21;	

		}
		
		/**
		 * USB ports
		 */
		public static class USB {
			public static final int RIGHT_JOYSTICK = 0;
			public static final int LEFT_JOYSTICK = 1;
			//public static final int DRIVER_GAMEPAD = 3;
			public static final int COPILOT_GAMEPAD = 2;
			public static final int BUTTON_BOX = 3;
			public static final int MAIN_JOYSTICK = 4;
		}
		
		/**
		 * PCM ports
		 */
		public static class PCM {
			/* 2017 robot
			public static final int INTAKE_IN = 0;
			public static final int INTAKE_OUT = 1;
			public static final int INTAKE_DOWN = 2;
			public static final int INTAKE_UP = 3;
			public static final int GEAR_IN = 5;
			public static final int GEAR_OUT = 4;
			public static final int BASIN_DOWN = 6;
			public static final int BASIN_UP = 7;*/
			
			// 2019 robot
			/*public static final int KICKER_OUT = 0;
			public static final int KICKER_IN = 1;		
			public static final int SUCKER_EXHALE = 2;
			public static final int SUCKER_INHALE= 3;
			public static final int EJECTOR_RETRACTED = 5;
			public static final int EJECTOR_EXTENDED = 4;			
			public static final int HOOK_UP = 7;
			public static final int HOOK_DOWN = 6;*/

			// 2020 robot
			/*public static final int GEAR_HIGH = 0;
			public static final int GEAR_LOW = 1;
			public static final int WINCH_STOPPER_STOPPED = 3;
			public static final int WINCH_STOPPER_FREE = 2; 
			public static final int WINCH_LOCK_LOCKED = 4;
			public static final int WINCH_LOCK_UNLOCKED = 5; 
			public static final int PUSHER_UP = 7; // cannot be used at same time as spinner raiser
			public static final int PUSHER_DOWN = 6;
			public static final int SPINNER_RAISER_UP = 7;
			public static final int SPINNER_RAISER_DOWN = 6;*/

			// 2022 robot
			/*public static final int GEAR_HIGH = 0;
			public static final int GEAR_LOW = 1;
			public static final int FRONT_ELBOWS_OPEN = 2;
			public static final int FRONT_ELBOWS_CLOSED = 3;
			public static final int REAR_ELBOWS_OPEN = 4;
			public static final int REAR_ELBOWS_CLOSED = 5;*/

			// 2023 robot
			/*public static final int GEAR_HIGH = 1;
			public static final int GEAR_LOW = 0;
			public static final int CLAW_CLOSED = 3;
			public static final int CLAW_OPEN = 2;
			public static final int BRAKE_ENGAGED = 5;
			public static final int BRAKE_RELEASED = 4;*/

			//2023 Off-season
			public static final int MOUTH_CLOSED = 0;
			public static final int MOUTH_OPEN = 1;
		}

		/**
		 * PWM ports
		 */
		public static class PWM {
			public static final int LED_STRIP = 9;
		}

		/**
		 * USB cameras
		 */
		public static class UsbCamera {
			public static final int FLOOR_CAMERA = 0;
			public static final int SHOOTER_CAMERA = 1;
			public static final int TOP_CAMERA = 2;
		}
}
