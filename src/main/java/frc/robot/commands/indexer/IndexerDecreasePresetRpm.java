package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Indexer;

public class IndexerDecreasePresetRpm extends InstantCommand {

	private Indexer indexer;

	public IndexerDecreasePresetRpm(Indexer indexer) {
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
		//System.out.println("IndexerDecreasePresetRpm: initialize");
		indexer.decreasePresetRps();
	}
}
