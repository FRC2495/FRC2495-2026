package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Indexer;

public class IndexerIncreasePresetRpm extends InstantCommand {

	private Indexer indexer;

	public IndexerIncreasePresetRpm(Indexer indexer) {
		this.indexer = indexer;
		addRequirements(indexer);
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		//System.out.println("IndexerIncreasePresetRpm: initialize");
		indexer.increasePresetRps();
	}
}
