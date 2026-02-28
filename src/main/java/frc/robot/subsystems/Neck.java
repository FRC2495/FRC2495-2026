package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import frc.robot.interfaces.*;
import frc.robot.RobotContainer;


/**
 * The {@code Neck} class contains fields and methods pertaining to the function of the neck.
 * 
 * The neck is responsible for controlling the angle of the intake.
 */
public class Neck extends SubsystemBase implements INeck {
	
	// general settings
	static final int TIMEOUT_MS = 15000;


	public static final int TICKS_PER_REVOLUTION = 2048; // FX Integrated Sensor = 2048 units per rotation

	public static final int ANGLE_TO_MIDWAY_REVS = 90000/TICKS_PER_REVOLUTION; // we divide by ticks per revolution to convert the ticks unit to revolutions
	public static final int ANGLE_TO_TRAVEL_REVS = 180000/TICKS_PER_REVOLUTION; // we divide by ticks per revolution to convert the ticks unit to revolutions
	
	/*
	!!! VIRTUAL_HOME_OFFSET_TICKS is important for moving up,     !!!
	!!! if this is changed make sure to check to see if moveUp() works !!!
	(it's used as an error margin for moving up, since we can't reliably check when it's up)
	*/
	static final double VIRTUAL_HOME_OFFSET_TICKS = -4000; // position of virtual home compared to physical home
	static final double VIRTUAL_HOME_OFFSET_REVS = VIRTUAL_HOME_OFFSET_TICKS / TICKS_PER_REVOLUTION;
	
	static final double MAX_PCT_OUTPUT = 1.0; // ~full speed
	
	static final int TALON_TIMEOUT_MS = 20;
	
	// move settings
	static final int PRIMARY_PID_LOOP = 0;
	
	static final int SLOT_0 = 0;
	
	static final double REDUCED_PCT_OUTPUT = 0.7;
	static final double SUPER_REDUCED_PCT_OUTPUT = 0.5;
	static final double HOMING_PCT_OUTPUT = 0.9;
	
	static final double MOVE_PROPORTIONAL_GAIN = 0.5; // An error of 1 rotation results in 0.5 V output
	static final double MOVE_INTEGRAL_GAIN = 0.0; // No output for integrated error
	static final double MOVE_DERIVATIVE_GAIN = 0.1; // Output is reduced by 0.1 V for every 1 rotation per second of error change
	
	static final double REV_THRESH = 1; // we are on target if we are within 1 revolution of the target (we can adjust this if needed)
	public static final double RPS_THRESH = 1; // we are stalled if we are moving less than 1 revolution per second (we can adjust this if needed)
	
	private static final int MOVE_ON_TARGET_MINIMUM_COUNT= 20; // number of times/iterations we need to be on target to really be on target

	private static final int MOVE_STALLED_MINIMUM_COUNT = MOVE_ON_TARGET_MINIMUM_COUNT * 2 + 30; // number of times/iterations we need to be stalled to really be stalled
	
	// variables
	boolean isMoving;
	boolean isMovingUp;
	boolean isReallyStalled;
	boolean isHoming;
	
	TalonFX neck;
	TalonFX neck_follower;

	TalonFXConfiguration neckConfig;

	DutyCycleOut neckStopOut = new DutyCycleOut(0);
	DutyCycleOut neckHomeOut = new DutyCycleOut(HOMING_PCT_OUTPUT);
	DutyCycleOut neckReducedOut = new DutyCycleOut(REDUCED_PCT_OUTPUT);

	PositionDutyCycle neckHomePosition  = new PositionDutyCycle(0);
	PositionDutyCycle neckMidwayPosition = new PositionDutyCycle(-ANGLE_TO_MIDWAY_REVS);
	PositionDutyCycle neckUpPosition = new PositionDutyCycle(-ANGLE_TO_TRAVEL_REVS);
	PositionDutyCycle neckVirtualHomePosition = new PositionDutyCycle(-VIRTUAL_HOME_OFFSET_REVS);
	
	double tac;

	private int onTargetCount; // counter indicating how many times/iterations we were on target
	private int stalledCount; // counter indicating how many times/iterations we were stalled	

	/**
 	* The {@code Neck} class contains fields and methods pertaining to the function of the neck.
	*/	
	public Neck(TalonFX neck_in, TalonFX neck_follower_in) {
		neck = neck_in;
		neck_follower = neck_follower_in;
		

		neckConfig = new TalonFXConfiguration();

		// Mode of operation during Neutral output may be set by using the setNeutralMode() function.
		// As of right now, there are two options when setting the neutral mode of a motor controller,
		// brake and coast.	
		neckConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		
		// Sensors for motor controllers provide feedback about the position, velocity, and acceleration
		// of the system using that motor controller.
		neckConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 

		neckConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        neckConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        neckConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
		
		//Enable reverse limit switches
		neckConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        neckConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        neckConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

		// this will reset the encoder automatically when at or past the reverse limit sensor
		neckConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
		neckConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;

		// Motor controller output direction can be set by calling the setInverted() function as seen below.
		// Note: Regardless of invert value, the LEDs will blink green when positive output is requested (by robot code or firmware closed loop).
		// Only the motor leads are inverted. This feature ensures that sensor phase and limit switches will properly match the LED pattern
		// (when LEDs are green => forward limit switch and soft limits are being checked). 	
		neckConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // change value or comment out if needed

		//setPIDParameters();
		var slot0Configs = neckConfig.Slot0;
		slot0Configs.kS = 0;
		slot0Configs.kV = 0;
		slot0Configs.kP = MOVE_PROPORTIONAL_GAIN;
		slot0Configs.kI = MOVE_INTEGRAL_GAIN;
		slot0Configs.kD = MOVE_DERIVATIVE_GAIN;
		
		// set peak output to max in case if had been reduced previously
		setPeakOutputs(REDUCED_PCT_OUTPUT);

		StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = neck.getConfigurator().apply(neckConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
		
		status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = neck_follower.getConfigurator().apply(neckConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

		// The follower feature allows the motor controllers to mimic another motor controller's output.
		neck_follower.setControl(new Follower(neck.getDeviceID(), MotorAlignmentValue.Opposed)); // sets the follower to follow the master

		// Motor controllers that are followers can set Status 1 and Status 2 to 255ms(max) using setStatusFramePeriod.
		// The Follower relies on the master status frame allowing its status frame to be slowed without affecting performance.
		// This is a useful optimization to manage CAN bus utilization.
		neck_follower.optimizeBusUtilization();

		isMoving = false;
		isMovingUp = false;
		isReallyStalled = false;
		stalledCount = 0;	
	}

	@Override
	public void periodic() {
		// Put code here to be run every loop

	}

	// homes the hinge
	// we go down slowly until we hit the limit switch.
	public void home() {
		neck.setControl(neckHomeOut); // we start moving down
		
		isHoming = true;
	}

	// this method need to be called to assess the homing progress
	// (and it takes care of going to step 2 if needed)
	public boolean checkHome() {
		if (isHoming) {
			isHoming = !getForwardLimitSwitchState(); // we are not done until we reach the switch

			if (!isHoming) {
				System.out.println("You have reached the home.");
				neck.setControl(neckStopOut); // turn power off
			}
		}

		return isHoming();
	}
	
	// This method should be called to assess the progress of a move
	public boolean tripleCheckMove() {
		if (isMoving) {
			
			double error = neck.getClosedLoopError().getValueAsDouble();
			//System.out.println("Neck moving error: " + Math.abs(error));
			
			boolean isOnTarget = (Math.abs(error) < REV_THRESH);
			
			if (isOnTarget) { // if we are on target in this iteration 
				onTargetCount++; // we increase the counter
			} else { // if we are not on target in this iteration
				if (onTargetCount > 0) { // even though we were on target at least once during a previous iteration
					onTargetCount = 0; // we reset the counter as we are not on target anymore
					System.out.println("Triple-check failed (neck moving).");
				} else {
					// we are definitely moving
				}
			}
			
			if (onTargetCount > MOVE_ON_TARGET_MINIMUM_COUNT) { // if we have met the minimum
				isMoving = false;
			}
			
			if (!isMoving) {
				System.out.println("You have reached the target (neck moving).");

				if (isMovingUp) {
					stay();
				} else {
					stop();
					//stay();
				}
			}
		}
		return isMoving; 
	}

	// return if drivetrain might be stalled
	public boolean tripleCheckIfStalled() {
		if (isMoving) {
			
			double velocity = getEncoderVelocity();
			
			boolean isStalled = (Math.abs(velocity) < RPS_THRESH); //TICK_PER_100MS_THRESH
			
			if (isStalled) { // if we are stalled in this iteration 
				stalledCount++; // we increase the counter
			} else { // if we are not stalled in this iteration
				if (stalledCount > 0) { // even though we were stalled at least once during a previous iteration
					stalledCount = 0; // we reset the counter as we are not stalled anymore
					System.out.println("Triple-check failed (detecting stall).");
				} else {
					// we are definitely not stalled
					
					//System.out.println("moving velocity : " + velocity);
				}
			}
			
			if (isMoving && stalledCount > MOVE_STALLED_MINIMUM_COUNT) { // if we have met the minimum
				isReallyStalled = true;
			}
					
			if (isReallyStalled) {
				System.out.println("WARNING: Stall detected!");
				stop(); // WE STOP IF A STALL IS DETECTED				 
			}
		}
		
		return isReallyStalled;
	}

	public int getEncoderVelocity() {
		return (int) neck.getVelocity().getValueAsDouble();
	}
	
	public void moveUp() {	

		//setPIDParameters();
		System.out.println("Moving Up");
		
		setPeakOutputs(REDUCED_PCT_OUTPUT);

		tac = neckUpPosition.Position;
		neck.setControl(neckUpPosition);
		
		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void moveCustom(double revs) {	

		//setPIDParameters();
		//System.out.println("Moving Custom");
		
		setPeakOutputs(REDUCED_PCT_OUTPUT);
		
		tac = revs;
		neck.setControl(new PositionDutyCycle(revs));
		
		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void moveMidway() {	

		//setPIDParameters();
		System.out.println("Moving to Midway");
		
		setPeakOutputs(REDUCED_PCT_OUTPUT);

		tac = neckMidwayPosition.Position;
		neck.setControl(neckMidwayPosition);
		
		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void moveDown() {
		
		//setPIDParameters();
		System.out.println("Moving Down");
		
		setPeakOutputs(SUPER_REDUCED_PCT_OUTPUT);

		tac = neckVirtualHomePosition.Position;
		neck.setControl(neckVirtualHomePosition);
		
		isMoving = true;
		isMovingUp = false;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public double getEncoderPosition() {
		return neck.getPosition().getValueAsDouble();
	}

	public void stay() {	 		
		isMoving = false;		
		isMovingUp = false;
		isHoming = false;
	}
	
	public void stop() {	 

		neck.setControl(neckStopOut);
		
		isMoving = false;
		isMovingUp = false;	
		isHoming = false;	
		
		setPeakOutputs(MAX_PCT_OUTPUT); // we undo what me might have changed
	}	
	
	/*private void setPIDParameters() {		
		//neck.configAllowableClosedloopError(SLOT_0, TALON_TICK_THRESH, TALON_TIMEOUT_MS);
		
		// P is the proportional gain. It modifies the closed-loop output by a proportion (the gain value)
		// of the closed-loop error.
		// P gain is specified in output unit per error unit.
		// When tuning P, it's useful to estimate your starting value.
		// If you want your mechanism to drive 50% output when the error is 4096 (one rotation when using CTRE Mag Encoder),
		// then the calculated Proportional Gain would be (0.50 X 1023) / 4096 = ~0.125.
		
		// I is the integral gain. It modifies the closed-loop output according to the integral error
		// (summation of the closed-loop error each iteration).
		// I gain is specified in output units per integrated error.
		// If your mechanism never quite reaches your target and using integral gain is viable,
		// start with 1/100th of the Proportional Gain.
		
		// D is the derivative gain. It modifies the closed-loop output according to the derivative error
		// (change in closed-loop error each iteration).
		// D gain is specified in output units per derivative error.
		// If your mechanism accelerates too abruptly, Derivative Gain can be used to smooth the motion.
		// Typically start with 10x to 100x of your current Proportional Gain.

		// Feed-Forward is typically used in velocity and motion profile/magic closed-loop modes.
		// F gain is multiplied directly by the set point passed into the programming API.
		// The result of this multiplication is in motor output units [-1023, 1023]. This allows the robot to feed-forward using the target set-point.
		// In order to calculate feed-forward, you will need to measure your motor's velocity at a specified percent output
		// (preferably an output close to the intended operating range).
	}*/

	public void setPeakOutputs(double peakOutput)
	{
		neckConfig.MotorOutput.PeakForwardDutyCycle = peakOutput;
		neckConfig.MotorOutput.PeakReverseDutyCycle = -peakOutput;
	}

	public boolean isHoming() {
		return isHoming;
	}

	public boolean isMoving() {
		return isMoving;
	}

	public boolean isMovingUp() {
		return isMovingUp;
	}
	
	public boolean isUp() {
		return Math.abs(getEncoderPosition()) > ANGLE_TO_TRAVEL_REVS * 2/3;
	}
	
	public boolean isDown() {
		return Math.abs(getEncoderPosition()) < ANGLE_TO_TRAVEL_REVS * 1/3;
	}
	
	public boolean isMidway() {
		return !isUp() && !isDown();
	}

	public boolean isDangerous() {
		return isUp();
	}

	// return if stalledF
	public boolean isStalled() {
		return isReallyStalled;
	}	
	
	// for debug purpose only
	public void joystickControl(Joystick joystick)
	{
		if (!isMoving) // if we are already doing a move we don't take over
		{
			neck.setControl(neckReducedOut.withOutput(+joystick.getY()));
		}
	}	

	public void gamepadControl(XboxController gamepad)
	{
		if (!isMoving) // if we are already doing a move we don't take over
		{
			neck.setControl(neckReducedOut.withOutput(+MathUtil.applyDeadband(gamepad.getRightY(),RobotContainer.GAMEPAD_AXIS_THRESHOLD)*0.6)); // adjust sign if desired
		}
	}

	public double getTarget() {
		return tac;
	}

	// returns the state of the limit switch
	public boolean getReverseLimitSwitchState() {
		return neck.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
	}

	public boolean getForwardLimitSwitchState() {
		return neck.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
	}

	// MAKE SURE THAT YOU ARE NOT IN A CLOSED LOOP CONTROL MODE BEFORE CALLING THIS METHOD.
	// OTHERWISE THIS IS EQUIVALENT TO MOVING TO THE DISTANCE TO THE CURRENT ZERO IN REVERSE! 
	public void resetEncoder() {
		neck.setControl(neckStopOut);
		neck.setPosition(0, TALON_TIMEOUT_MS);
	}

}
