package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Indexer;

/**
 *
 */
public class IndexerIndexCustom extends Command {

	private Indexer indexer;

	private double custom_rpm;

	public IndexerIndexCustom(Indexer indexer_in, double custom_rpm_in) {

		this.indexer = indexer_in;
		this.custom_rpm = custom_rpm_in;

		addRequirements(
			indexer);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		//System.out.println("IndexerIndexCustom: initialize");
		indexer.indexCustom(custom_rpm);
	}

}

