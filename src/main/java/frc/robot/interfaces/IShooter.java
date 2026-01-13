package frc.robot.interfaces;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IShooter extends Subsystem {
	
	public void shootHigh();
	
	public void shootLow();

	public void stop();
		
	// NOTE THAT THIS METHOD WILL IMPACT BOTH OPEN AND CLOSED LOOP MODES
	public void setPeakOutputs(double peakOutput);
	
	public boolean isShooting();

	// for debug purpose only
	public void joystickControl(Joystick joystick);
}










