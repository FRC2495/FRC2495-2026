package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Indexer;

/**
 *
 */
public class IndexerIndexHigh extends Command {

	private Indexer indexer;

	public IndexerIndexHigh(Indexer indexer_in) {

		this.indexer = indexer_in;
		addRequirements(indexer);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		//System.out.println("IndexerIndexHigh: initialize");
		indexer.indexHigh();
	}
}
