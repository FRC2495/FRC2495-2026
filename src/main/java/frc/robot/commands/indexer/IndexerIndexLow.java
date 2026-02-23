package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Indexer;

/**
 *
 */
public class IndexerIndexLow extends Command {

	private Indexer indexer;

	public IndexerIndexLow(Indexer indexer_in) {

		this.indexer = indexer_in;

		addRequirements(
			indexer);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		//System.out.println("IndexerIndexLow: initialize");
		indexer.indexLow();
	}
}
