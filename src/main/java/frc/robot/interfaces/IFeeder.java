package frc.robot.interfaces;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IFeeder extends Subsystem {
	
	public void feedHigh();

	public void feedLow();

	public void stop();
		
	// NOTE THAT THIS METHOD WILL IMPACT BOTH OPEN AND CLOSED LOOP MODES
	public void setPeakOutputs(double peakOutput);
	
	public boolean isFeeding();

	// for debug purpose only
	public void joystickControl(Joystick joystick);
}










