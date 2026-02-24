package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Feeder;

/**
 *
 */
public class FeederFeedCustom extends Command {

	private Feeder feeder;

	private double custom_rpm;

	public FeederFeedCustom(Feeder feeder_in, double custom_rpm_in) {

		this.feeder = feeder_in;
		this.custom_rpm = custom_rpm_in;

		addRequirements(
			feeder);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		//System.out.println("FeederIndexCustom: initialize");
		feeder.feedCustom(custom_rpm);
	}

}

