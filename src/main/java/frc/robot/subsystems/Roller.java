/**
 * 
 */
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.interfaces.*;
//import frc.robot.commands.roller.*;
//import frc.robot.sensors.Sonar;


/**
 * The {@code Roller} class contains fields and methods pertaining to the function of the roller.
 * 
 * The roller on the intake is responsible for collecting fuels. 
 */
public class Roller extends SubsystemBase implements IRoller{
	/**
	 * 
	 */
	static final double MAX_PCT_OUTPUT = 1.0;
	static final double ALMOST_MAX_PCT_OUTPUT = 0.9;
	static final double HALF_PCT_OUTPUT = 0.5;
	static final double REDUCED_PCT_OUTPUT = 0.6;
	
	static final int WAIT_MS = 1000;
	static final int TIMEOUT_MS = 5000;

	static final int SECONDS_PER_MINUTE = 60;

	static final int TALON_TIMEOUT_MS = 20;

	private double custom_rps = ROLL_LOW_RPS; // custom rps that can be set by the user
	private double presetRps = ROLL_HIGH_RPS; // preset rps
	
	TalonFX rollerMaster;
	TalonFX rollerFollower;

	TalonFXConfiguration rollerMasterConfig;

	DutyCycleOut rollerStopOut = new DutyCycleOut(0);
	DutyCycleOut rollerRedOut = new DutyCycleOut(REDUCED_PCT_OUTPUT);
	DutyCycleOut rollerMaxOut = new DutyCycleOut(MAX_PCT_OUTPUT);

	double targetVelocity = (ROLL_HIGH_RPS);
	double targetLowVelocity = (ROLL_LOW_RPS);
	double targetCustomVelocity = (custom_rps);
	double targetPresetVelocity = (presetRps);

	private final VelocityVoltage rollerVelocity = new VelocityVoltage(0);

	boolean isRolling;
	boolean isReleasing;
	
	// index settings
	static final int PRIMARY_PID_LOOP = 0;

	static final int SLOT_0 = 0;

	static final double ROLL_PROPORTIONAL_GAIN = 0.1; // An error of 1 rotation per second results in 0.1 V output - increase up to 1 if you want it to be more aggressive, but be careful of oscillations 
	static final double ROLL_INTEGRAL_GAIN = 0.0001; // An error of 1 rotation per second sustained for 1 second results in 0.0001 V output - reduce if you see oscillations, increase if you see steady state error (i.e. the roller is running at a velocity slightly below the target velocity)
	static final double ROLL_DERIVATIVE_GAIN = 0.001; // A change in error of 1 rotation per second per second results in 0.001 V output - increase if you see the roller accelerating too abruptly, but be careful of oscillations
	static final double ROLL_STATIC_FEED_FORWARD = 0.1; // To account for friction, add 0.1 V of static feedforward - reduce if you see the roller overshooting the target velocity, increase if you see the roller struggling to reach the target velocity
	//static final double ROLL_VELOCITY_FEED_FORWARD = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.333 = 0.12 volts / rotation per second
	static final double ROLL_VELOCITY_FEED_FORWARD = 0.11; // Falcon 500 is a ~530 kV motor, 530 rpm per V = 8.833 rps per V, 1/8.833 = 0.11 volts / rotation per second
	static final double ROLL_HIGH_RPS = 5000.0 / SECONDS_PER_MINUTE;
	static final double ROLL_LOW_RPS = 1500.0 / SECONDS_PER_MINUTE;

	static final double PRESET_DELTA_RPS = 100.0 / SECONDS_PER_MINUTE; // by what we increase/decrease by default
	
	
	public Roller(TalonFX rollerMaster_in, TalonFX rollerFollower_in) {
		
		rollerMaster = rollerMaster_in;
		rollerFollower = rollerFollower_in;

		rollerMasterConfig = new TalonFXConfiguration();
		
		// Mode of operation during Neutral output may be set by using the setNeutralMode() function.
		// As of right now, there are two options when setting the neutral mode of a motor controller,
		// brake and coast.
		rollerMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		// Sensors for motor controllers provide feedback about the position, velocity, and acceleration
		// of the system using that motor controller.
		rollerMasterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 
		// Motor controller output direction can be set by calling the setInverted() function as seen below.
		// Note: Regardless of invert value, the LEDs will blink green when positive output is requested (by robot code or firmware closed loop).
		// Only the motor leads are inverted. This feature ensures that sensor phase and limit switches will properly match the LED pattern
		// (when LEDs are green => forward limit switch and soft limits are being checked).
		rollerMasterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // change value or comment out if needed
		
		// set peak output to max in case if had been reduced previously
		setPeakOutputs(MAX_PCT_OUTPUT);

		var slot0Configs = rollerMasterConfig.Slot0;
		slot0Configs.kS = ROLL_STATIC_FEED_FORWARD; // volts output at 0 velocity error, typically used to overcome static friction
		slot0Configs.kV = ROLL_VELOCITY_FEED_FORWARD; // https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html
		slot0Configs.kP = ROLL_PROPORTIONAL_GAIN; // An error of 1 rotation per second results in ROLL_PROPORTIONAL_GAIN volts output
		slot0Configs.kI = ROLL_INTEGRAL_GAIN; // An error of 1 rotation per second sustained for 1 second results in ROLL_INTEGRAL_GAIN volts output
		slot0Configs.kD = ROLL_DERIVATIVE_GAIN; // A change in error of 1 rotation per second per second results in ROLL_DERIVATIVE_GAIN volts output

		StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = rollerMaster.getConfigurator().apply(rollerMasterConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

		status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = rollerFollower.getConfigurator().apply(rollerMasterConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

		// The follower feature allows the motor controllers to mimic another motor controller's output.
		rollerFollower.setControl(new Follower(rollerMaster.getDeviceID(), MotorAlignmentValue.Opposed)); // sets the follower to follow the master

		// Motor controllers that are followers can set Status 1 and Status 2 to 255ms(max) using setStatusFramePeriod.
		// The Follower relies on the master status frame allowing its status frame to be slowed without affecting performance.
		// This is a useful optimization to manage CAN bus utilization.
		rollerFollower.optimizeBusUtilization();
	}
	
	/*@Override
	public void initDefaultCommand() {
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

		// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

		// Set the default command for a subsystem here.
		setDefaultCommand(new FeederStop());
	}*/

	@Override
	public void periodic() {
		// Put code here to be run every loop
	}

	public void rollIn() {
		//setPIDParameters();
		setPeakOutputs(MAX_PCT_OUTPUT); //MAX_PCT_OUTPUT //this has a global impact, so we reset in stop()

		rollerMaster.setControl(rollerVelocity.withVelocity(+targetVelocity));
		
		isRolling = true;
		isReleasing = false;
	}

	public void rollInLowRpm() {
		//setPIDParameters();
		setPeakOutputs(MAX_PCT_OUTPUT); //this has a global impact, so we reset in stop()

		rollerMaster.setControl(rollerVelocity.withVelocity(+targetLowVelocity));
		
		isRolling = true;
		isReleasing = false;
	}	

	public void rollOut() {
		//setPIDParameters();
		setPeakOutputs(MAX_PCT_OUTPUT); //MAX_PCT_OUTPUT //this has a global impact, so we reset in stop()

		rollerMaster.setControl(rollerVelocity.withVelocity(-targetVelocity));
		
		isRolling = false;
		isReleasing = true;
	}

	public void rollOutLowRpm() {
		//setPIDParameters();
		setPeakOutputs(MAX_PCT_OUTPUT); //this has a global impact, so we reset in stop()

		rollerMaster.setControl(rollerVelocity.withVelocity(-targetLowVelocity));
		
		isRolling = false;
		isReleasing = true;
	}

	public void rollCustom(double custom_rps) {
		//setPIDParameters();
		setPeakOutputs(MAX_PCT_OUTPUT); //this has a global impact, so we reset in stop()

		rollerMaster.setControl(rollerVelocity.withVelocity(-custom_rps));
		
		isRolling = true;
		isReleasing = false;
	}

	public void rollPreset() {
		//setPIDParameters();
		setPeakOutputs(MAX_PCT_OUTPUT); //this has a global impact, so we reset in stop()

		rollerMaster.setControl(rollerVelocity.withVelocity(-targetPresetVelocity));
		
		isRolling = true;
		isReleasing = false;
	}

	public void increasePresetRps()
	{
		presetRps += PRESET_DELTA_RPS;
	}

	public void decreasePresetRps()
	{
		presetRps -= PRESET_DELTA_RPS;
	}

	public double getPresetRps()
	{
		return presetRps;
	}
	
	public void stop() {
		rollerMaster.setControl(rollerStopOut);

		isRolling = false;
		isReleasing = false;

		setPeakOutputs(MAX_PCT_OUTPUT); // we undo what me might have changed
	}
	
	/*public void setPIDParameters()
	{
		//shooterMaster.configAllowableClosedloopError(SLOT_0, TICK_PER_100MS_THRESH, TALON_TIMEOUT_MS);
		
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
		
	// NOTE THAT THIS METHOD WILL IMPACT BOTH OPEN AND CLOSED LOOP MODES
	public void setPeakOutputs(double peakOutput)
	{
		rollerMasterConfig.MotorOutput.PeakForwardDutyCycle = peakOutput;
		rollerMasterConfig.MotorOutput.PeakReverseDutyCycle = -peakOutput;
	}
	
	public boolean isRolling(){
		return isRolling;
	}

	public boolean isReleasing(){
		return isReleasing;
	}

	// for debug purpose only
	public void joystickControl(Joystick joystick)
	{
		rollerMaster.setControl(rollerRedOut.withOutput(+joystick.getY()));
	}

	// in RPS
	public int getEncoderVelocity() {
		return (int) rollerMaster.getVelocity().getValueAsDouble();
	}

	// in revolutions per minute
	public int getRpm() {
		return (int) (rollerMaster.getVelocity().getValueAsDouble()*SECONDS_PER_MINUTE); 
	}
}










