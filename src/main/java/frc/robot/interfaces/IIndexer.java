package frc.robot.interfaces;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IIndexer extends Subsystem {
	
	public void indexHigh();

	public void indexLow();

	public void stop();
		
	// NOTE THAT THIS METHOD WILL IMPACT BOTH OPEN AND CLOSED LOOP MODES
	public void setPeakOutputs(double peakOutput);
	
	public boolean isIndexing();

	// for debug purpose only
	public void joystickControl(Joystick joystick);
}










