package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Feeder;

/**
 *
 */
public class FeederFeedPreset extends Command {

	private Feeder feeder;

	public FeederFeedPreset(Feeder feeder_in) {

		this.feeder = feeder_in;

		addRequirements(
			feeder);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		//System.out.println("FeederFeedPreset: initialize");
		feeder.feedPreset();
	}
}
