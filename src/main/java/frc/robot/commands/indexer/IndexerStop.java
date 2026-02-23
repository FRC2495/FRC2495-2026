package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Indexer;

/**
 *
 */
public class IndexerStop extends InstantCommand {

	private Indexer indexer;

	public IndexerStop(Indexer indexer_in) {

		this.indexer = indexer;
		addRequirements(indexer);
	}

	// Called once when this command runs
	@Override
	public void initialize() {
		//System.out.println("IndexerStop: initialize");
		indexer.stop();
	}

}
