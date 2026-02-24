package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Feeder;

/**
 *
 */
public class FeederStop extends InstantCommand {

	private Feeder feeder;

	public FeederStop(Feeder feeder_in) {
		this.feeder = feeder_in;
		addRequirements(feeder);
	}

	// Called once when this command runs
	@Override
	public void initialize() {
		//System.out.println("FeederStop: initialize");
		feeder.stop();
	}

}
