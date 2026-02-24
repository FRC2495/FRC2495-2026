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
//import frc.robot.commands.shooter.*;
//import frc.robot.sensors.Sonar;
import frc.robot.sensors.SwitchedCamera;
import frc.robot.Ports;


/**
 * The {@code Shooter} class contains fields and methods pertaining to the function of the shooter.
 */
public class Shooter extends SubsystemBase implements IShooter{
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

	private double custom_rps = SHOOT_LOW_RPS; // custom rps that can be set by the user
	private double presetRps = SHOOT_HIGH_RPS; // preset rps
	
	TalonFX shooterMaster;
	TalonFX shooterFollower;

	TalonFXConfiguration shooterMasterConfig;

	DutyCycleOut shooterStopOut = new DutyCycleOut(0);
	DutyCycleOut shooterRedOut = new DutyCycleOut(REDUCED_PCT_OUTPUT);
	DutyCycleOut shooterMaxOut = new DutyCycleOut(MAX_PCT_OUTPUT);

	double targetVelocity = (SHOOT_HIGH_RPS);
	double targetLowVelocity = (SHOOT_LOW_RPS);
	double targetCustomVelocity = (custom_rps);
	double targetPresetVelocity = (presetRps);

	private final VelocityVoltage shooterVelocity = new VelocityVoltage(0);

	boolean isShooting;
	
	// shoot settings
	static final int PRIMARY_PID_LOOP = 0;

	static final int SLOT_0 = 0;

	static final double SHOOT_PROPORTIONAL_GAIN = 0.1; // An error of 1 rotation per second results in 0.1 V output - increase up to 1 if you want it to be more aggressive, but be careful of oscillations 
	static final double SHOOT_INTEGRAL_GAIN = 0.0001; // An error of 1 rotation per second sustained for 1 second results in 0.0001 V output - reduce if you see oscillations, increase if you see steady state error (i.e. the shooter is running at a velocity slightly below the target velocity)
	static final double SHOOT_DERIVATIVE_GAIN = 0.001; // A change in error of 1 rotation per second per second results in 0.001 V output - increase if you see the shooter accelerating too abruptly, but be careful of oscillations
	static final double SHOOT_STATIC_FEED_FORWARD = 0.1; // To account for friction, add 0.1 V of static feedforward - reduce if you see the shooter overshooting the target velocity, increase if you see the shooter struggling to reach the target velocity
	static final double SHOOT_VELOCITY_FEED_FORWARD = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second


	static final double SHOOT_HIGH_RPS = 3500.0 / SECONDS_PER_MINUTE;
	static final double SHOOT_LOW_RPS = 1500.0 / SECONDS_PER_MINUTE;

	static final double PRESET_DELTA_RPS = 100.0 / SECONDS_PER_MINUTE; // by what we increase/decrease by default
	
	
	public Shooter(TalonFX shooterMaster_in, TalonFX shooterFollower_in) {
		
		shooterMaster = shooterMaster_in;
		shooterFollower = shooterFollower_in;

		shooterMasterConfig = new TalonFXConfiguration();
		
		// Mode of operation during Neutral output may be set by using the setNeutralMode() function.
		// As of right now, there are two options when setting the neutral mode of a motor controller,
		// brake and coast.
		shooterMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		// Sensors for motor controllers provide feedback about the position, velocity, and acceleration
		// of the system using that motor controller.
		shooterMasterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 

		// Motor controller output direction can be set by calling the setInverted() function as seen below.
		// Note: Regardless of invert value, the LEDs will blink green when positive output is requested (by robot code or firmware closed loop).
		// Only the motor leads are inverted. This feature ensures that sensor phase and limit switches will properly match the LED pattern
		// (when LEDs are green => forward limit switch and soft limits are being checked).
		shooterMasterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // change value or comment out if needed
		
		// set peak output to max in case if had been reduced previously
		setPeakOutputs(MAX_PCT_OUTPUT);

		var slot0Configs = shooterMasterConfig.Slot0;
		slot0Configs.kS = SHOOT_STATIC_FEED_FORWARD; // volts output at 0 velocity error, typically used to overcome static friction
		slot0Configs.kV = SHOOT_VELOCITY_FEED_FORWARD; // https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html
		slot0Configs.kP = SHOOT_PROPORTIONAL_GAIN; // An error of 1 rotation per second results in SHOOT_PROPORTIONAL_GAIN volts output
		slot0Configs.kI = SHOOT_INTEGRAL_GAIN; // An error of 1 rotation per second sustained for 1 second results in SHOOT_INTEGRAL_GAIN volts output
		slot0Configs.kD = SHOOT_DERIVATIVE_GAIN; // A change in error of 1 rotation per second per second results in SHOOT_DERIVATIVE_GAIN volts output

		StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = shooterMaster.getConfigurator().apply(shooterMasterConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

		shooterFollower.setControl(new Follower(shooterMaster.getDeviceID(), MotorAlignmentValue.Opposed)); // sets the follower to follow the master
	}
	
	/*@Override
	public void initDefaultCommand() {
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

		// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

		// Set the default command for a subsystem here.
		setDefaultCommand(new ShooterStop());
	}*/

	@Override
	public void periodic() {
		// Put code here to be run every loop
	}

	public void shootHigh() {
		SwitchedCamera.setUsbCamera(Ports.UsbCamera.SHOOTER_CAMERA);

		//setPIDParameters();
		setPeakOutputs(MAX_PCT_OUTPUT); //MAX_PCT_OUTPUT //this has a global impact, so we reset in stop()

		shooterMaster.setControl(shooterVelocity.withVelocity(targetVelocity));
		
		isShooting = true;
	}

	public void shootLow() {
		SwitchedCamera.setUsbCamera(Ports.UsbCamera.SHOOTER_CAMERA);

		//setPIDParameters();
		setPeakOutputs(MAX_PCT_OUTPUT); //this has a global impact, so we reset in stop()

		shooterMaster.setControl(shooterVelocity.withVelocity(targetLowVelocity));
		
		isShooting = true;
	}

	public void shootCustom(double custom_rps) {
		SwitchedCamera.setUsbCamera(Ports.UsbCamera.SHOOTER_CAMERA);

		//setPIDParameters();
		setPeakOutputs(MAX_PCT_OUTPUT); //this has a global impact, so we reset in stop()

		shooterMaster.setControl(shooterVelocity.withVelocity(custom_rps));
		
		isShooting = true;
	}

	public void shootPreset() {
		SwitchedCamera.setUsbCamera(Ports.UsbCamera.SHOOTER_CAMERA);

		//setPIDParameters();
		setPeakOutputs(MAX_PCT_OUTPUT); //this has a global impact, so we reset in stop()

		shooterMaster.setControl(shooterVelocity.withVelocity(targetPresetVelocity));
		
		isShooting = true;
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
		shooterMaster.setControl(shooterStopOut);

		isShooting = false;

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
		shooterMasterConfig.MotorOutput.PeakForwardDutyCycle = peakOutput;
		shooterMasterConfig.MotorOutput.PeakReverseDutyCycle = -peakOutput;
	}
	
	public boolean isShooting(){
		return isShooting;
	}

	// for debug purpose only
	public void joystickControl(Joystick joystick)
	{
		shooterMaster.setControl(shooterRedOut.withOutput(joystick.getY()));
	}

	// in RPS
	public int getEncoderVelocity() {
		return (int) shooterMaster.getVelocity().getValueAsDouble();
	}

	// in revolutions per minute
	public int getRpm() {
		return (int) (shooterMaster.getVelocity().getValueAsDouble()*SECONDS_PER_MINUTE);
	}
}










