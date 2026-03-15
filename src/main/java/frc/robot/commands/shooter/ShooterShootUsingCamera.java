package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;
import frc.robot.utils.Magic;

import frc.robot.vision.PhotonVisionSystem;

/**
 * A command that shoots the fuel using the camera to determine the distance to the hub and therefore the needed shooter RPM.
 */
public class ShooterShootUsingCamera extends Command {

	private Shooter shooter;
	private PhotonVisionSystem vision;

	public ShooterShootUsingCamera(Shooter shooter_in, PhotonVisionSystem vision) {

		this.shooter = shooter_in;
		this.vision = vision;

		addRequirements(
			shooter);

	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		//System.out.println("ShooterShootUsingCamera: initialize");

		double distance = vision.getDistanceToHub();  // will return 0.0 by convention if no target acquired
		//double angle = vision.getRotationToHub().getDegrees(); // angle call not atomic with distance call, but good enough for this use case

		double magic_rpm = distance!=0.0 ? Magic.getRpm(distance) : 3000; // if we have a valid distance, use it to calculate the rpm, otherwise just shoot at a default rpm

		shooter.shootCustom(magic_rpm);
	}
}
