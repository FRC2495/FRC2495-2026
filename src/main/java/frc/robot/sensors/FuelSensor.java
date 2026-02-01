// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;

/**
 * The {@code NoteSensor} class contains fields and methods pertaining to the function of the note sensor.
 */
public class FuelSensor extends Canandcolor {

	private double triggerValue; 

	public FuelSensor(int port, double triggerVal) {
		super(port);
		resetFactoryDefaults();

		this.triggerValue = triggerVal;
	}

	/**
	 * Returns the state of the coral sensor.
	 *
	 * @return the current state of the coral sensor.
	 */
	public boolean isTriggered() { // if the reported value is less than your trigger value, it returns true
        return isConnected() && getProximity() < triggerValue; // works because of shortcircuit eval
    }
}