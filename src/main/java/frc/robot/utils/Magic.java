package frc.robot.utils;

public class Magic {
	
	/**
	 * This method calculates the RPM needed to shoot a fuel into the hub based on the distance to the hub.
	 * 
	 * The distance is expected to be in meters, and the RPM is returned as a double.
	 */
	public static double getRpm(double distance)
	{
		return 2000 + distance * 500; // 2000 rpm at 0 meter, increases by 500 rpm per meter - TODO: tune this formula
	}
}

