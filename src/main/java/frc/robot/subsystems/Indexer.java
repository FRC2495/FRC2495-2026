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
//import frc.robot.commands.indexer.*;
//import frc.robot.sensors.Sonar;


/**
 * The {@code Indexer} class contains fields and methods pertaining to the function of the indexer.
 */
public class Indexer extends SubsystemBase implements IIndexer{
	/**
	 * 
	 */
	static final double MAX_PCT_OUTPUT = 1.0;
	static final double ALMOST_MAX_PCT_OUTPUT = 1.0;
	static final double HALF_PCT_OUTPUT = 0.5;
	static final double REDUCED_PCT_OUTPUT = 0.6;
	
	static final int WAIT_MS = 1000;
	static final int TIMEOUT_MS = 5000;

	static final int SECONDS_PER_MINUTE = 60;

	static final int TALON_TIMEOUT_MS = 20;

	private double custom_rps = INDEX_LOW_RPS; //TODO change value ?
	private double presetRps = INDEX_HIGH_RPS; // preset rps
	
	TalonFX indexerMaster;
	TalonFX indexerFollower;

	TalonFXConfiguration indexerMasterConfig;
	TalonFXConfiguration indexerFollowerConfig;

	DutyCycleOut indexerStopOut = new DutyCycleOut(0);
	DutyCycleOut indexerRedOut = new DutyCycleOut(REDUCED_PCT_OUTPUT);
	DutyCycleOut indexerMaxOut = new DutyCycleOut(MAX_PCT_OUTPUT);

	double targetVelocity = (INDEX_HIGH_RPS); // add * 12 for velocity voltage if needed
	double targetLowVelocity = (INDEX_LOW_RPS); // 1 revolution = TICKS_PER_ROTATION ticks, 1 min = 600 * 100 ms
	double targetCustomVelocity = (custom_rps); // old conversion : * FX_INTEGRATED_SENSOR_TICKS_PER_ROTATION / 600
	double targetPresetVelocity = (presetRps); //

	private final VelocityVoltage indexerVelocity = new VelocityVoltage(0);

	boolean isIndexing;
	
	// index settings
	static final int PRIMARY_PID_LOOP = 0;

	static final int SLOT_0 = 0;

	static final double INDEX_PROPORTIONAL_GAIN = 0.09; // An error of 1 rotation per second results in 0.09 V output - increase up to 0.9 if you want it to be more aggressive, but be careful of oscillations 
	static final double INDEX_INTEGRAL_GAIN = 0.002; // An error of 1 rotation per second sustained for 1 second results in 0.002 V output - reduce if you see oscillations, increase if you see steady state error (i.e. the indexer is running at a velocity slightly below the target velocity)
	static final double INDEX_DERIVATIVE_GAIN = 0.004; // A change in error of 1 rotation per second per second results in 0.004 V output - increase if you see the indexer accelerating too abruptly, but be careful of oscillations
	static final double INDEX_STATIC_FEED_FORWARD = 0.1; // To account for friction, add 0.1 V of static feedforward - reduce if you see the indexer overshooting the target velocity, increase if you see the indexer struggling to reach the target velocity
	static final double INDEX_VELOCITY_FEED_FORWARD = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second

	static final double INDEX_HIGH_RPS = 3500.0 / SECONDS_PER_MINUTE;
	static final double INDEX_LOW_RPS = 1500.0 / SECONDS_PER_MINUTE;

	static final double PRESET_DELTA_RPS = 100.0 / SECONDS_PER_MINUTE; // by what we increase/decrease by default
	
	
	public Indexer(TalonFX indexerMaster_in, TalonFX indexerFollower_in) {
		
		indexerMaster = indexerMaster_in;
		indexerFollower = indexerFollower_in;

		indexerMasterConfig = new TalonFXConfiguration();
		
		// Mode of operation during Neutral output may be set by using the setNeutralMode() function.
		// As of right now, there are two options when setting the neutral mode of a motor controller,
		// brake and coast.
		indexerMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		// Sensors for motor controllers provide feedback about the position, velocity, and acceleration
		// of the system using that motor controller.
		// Note: With Phoenix framework, position units are in the natural units of the sensor.
		// This ensures the best resolution possible when performing closed-loops in firmware.
		// CTRE Magnetic Encoder (relative/quadrature) =  4096 units per rotation
		// FX Integrated Sensor = 2048 units per rotation
		indexerMasterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 

		// Motor controller output direction can be set by calling the setInverted() function as seen below.
		// Note: Regardless of invert value, the LEDs will blink green when positive output is requested (by robot code or firmware closed loop).
		// Only the motor leads are inverted. This feature ensures that sensor phase and limit switches will properly match the LED pattern
		// (when LEDs are green => forward limit switch and soft limits are being checked).
		indexerMasterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // change value or comment out if needed
		
		// set peak output to max in case if had been reduced previously
		setPeakOutputs(MAX_PCT_OUTPUT);

		var slot0Configs = indexerMasterConfig.Slot0;
		slot0Configs.kS = INDEX_STATIC_FEED_FORWARD; // volts output at 0 velocity error, typically used to overcome static friction
		slot0Configs.kV = INDEX_VELOCITY_FEED_FORWARD; // https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html
		slot0Configs.kP = INDEX_PROPORTIONAL_GAIN; // An error of 1 rotation per second results in INDEX_PROPORTIONAL_GAIN volts output
		slot0Configs.kI = INDEX_INTEGRAL_GAIN; // An error of 1 rotation per second sustained for 1 second results in INDEX_INTEGRAL_GAIN volts output

		StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = indexerMaster.getConfigurator().apply(indexerMasterConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

		indexerFollower.setControl(new Follower(indexerMaster.getDeviceID(), MotorAlignmentValue.Aligned)); // sets the follower to follow the master, and ensures that the motors are aligned (i.e. if one motor is inverted, the other will be inverted as well)
	}
	
	/*@Override
	public void initDefaultCommand() {
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

		// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

		// Set the default command for a subsystem here.
		setDefaultCommand(new IndexerStop());
	}*/

	@Override
	public void periodic() {
		// Put code here to be run every loop
	}

	public void indexHigh() {
		//setPIDParameters();
		setPeakOutputs(MAX_PCT_OUTPUT); //MAX_PCT_OUTPUT //this has a global impact, so we reset in stop()

		indexerMaster.setControl(indexerVelocity.withVelocity(targetVelocity));
		
		isIndexing = true;
	}

	public void indexLow() {
		//setPIDParameters();
		setPeakOutputs(MAX_PCT_OUTPUT); //this has a global impact, so we reset in stop()

		indexerMaster.setControl(indexerVelocity.withVelocity(targetLowVelocity));
		
		isIndexing = true;
	}

	public void indexCustom(double custom_rps) {
		//setPIDParameters();
		setPeakOutputs(MAX_PCT_OUTPUT); //this has a global impact, so we reset in stop()

		indexerMaster.setControl(indexerVelocity.withVelocity(custom_rps));
		
		isIndexing = true;
	}

	public void indexPreset() {
		//setPIDParameters();
		setPeakOutputs(MAX_PCT_OUTPUT); //this has a global impact, so we reset in stop()

		indexerMaster.setControl(indexerVelocity.withVelocity(targetPresetVelocity));
		
		isIndexing = true;
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
		indexerMaster.setControl(indexerStopOut);

		isIndexing = false;

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

		// set slot 0 gains and leave every other config factory-default
		var slot0Configs = shooterMasterConfig.Slot0;
		slot0Configs.kV = SHOOT_FEED_FORWARD * 2048 / 1023 / 10; // https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html
		slot0Configs.kP = SHOOT_PROPORTIONAL_GAIN * 2048 / 1023 / 10;
		slot0Configs.kI = SHOOT_INTEGRAL_GAIN * 2048 / 1023 * 1000 / 10;
		slot0Configs.kD = SHOOT_DERIVATIVE_GAIN * 2048 / 1023 / 1000 / 10;
		//slot0Configs.kS = SHOOT_DERIVATIVE_GAIN; //TODO change value

		shooterMaster.config_kP(SLOT_0, SHOOT_PROPORTIONAL_GAIN, TALON_TIMEOUT_MS);
		shooterMaster.config_kI(SLOT_0, SHOOT_INTEGRAL_GAIN, TALON_TIMEOUT_MS);
		shooterMaster.config_kD(SLOT_0, SHOOT_DERIVATIVE_GAIN, TALON_TIMEOUT_MS);	
		shooterMaster.config_kF(SLOT_0, SHOOT_FEED_FORWARD, TALON_TIMEOUT_MS);
	}*/
		
	// NOTE THAT THIS METHOD WILL IMPACT BOTH OPEN AND CLOSED LOOP MODES
	public void setPeakOutputs(double peakOutput)
	{
		indexerMasterConfig.MotorOutput.PeakForwardDutyCycle = peakOutput;
		indexerMasterConfig.MotorOutput.PeakReverseDutyCycle = -peakOutput;
	}
	
	public boolean isIndexing(){
		return isIndexing;
	}

	// for debug purpose only
	public void joystickControl(Joystick joystick)
	{
		indexerMaster.setControl(indexerRedOut.withOutput(joystick.getY()));
	}

	// in units per 100 ms
	public int getEncoderVelocity() {
		return (int) indexerMaster.getVelocity().getValueAsDouble();
	}

	// in revolutions per minute
	public int getRpm() {
		return (int) (indexerMaster.getVelocity().getValueAsDouble()*SECONDS_PER_MINUTE);  // 1 min = 600 * 100 ms, 1 revolution = TICKS_PER_ROTATION ticks 
	}
}










