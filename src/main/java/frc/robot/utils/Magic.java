package frc.robot.utils;

import edu.wpi.first.math.util.Units;

public class Magic {
	
	/**
	 * This method calculates the RPM needed to shoot a fuel into the hub based on the distance to the hub.
	 * 
	 * The distance is expected to be in meters, and the RPM is returned as a double.
	 */
	public static double getRpm(double distance)
	{
		//return 2000 + distance * 500; // 2000 rpm at 0 meter, increases by 500 rpm per meter - TODO: tune this formula

		//double distanceInFeet = Units.metersToFeet(distance); // convert meters to feet

		//return 2.6 * Math.pow(distanceInFeet, 2) + 16.8*distanceInFeet + 2371 +500; // Example quadratic formula: RPM = 12.7 * (distance in feet)^2 - 57.7 * (distance in feet) + 2660 - TODO: tune this formula

		return 360 * distance + 1580;
	}
}

