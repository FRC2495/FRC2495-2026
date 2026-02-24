package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Feeder;

/**
 *
 */
public class FeederFeedLow extends Command {

	private Feeder feeder;

	public FeederFeedLow(Feeder feeder_in) {

		this.feeder = feeder_in;

		addRequirements(
			feeder);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		//System.out.println("FeederFeedLow: initialize");
		feeder.feedLow();
	}
}
