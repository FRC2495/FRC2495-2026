package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Indexer;

/**
 * Add your docs here.
 */
public class IndexerTimedIndexHighNoStop extends WaitCommand {
	/**
	 * Add your docs here.
	 */
	private Indexer indexer;

	public IndexerTimedIndexHighNoStop(Indexer indexer, double timeout) {
		super(timeout);
		this.indexer = indexer;
		addRequirements(indexer);
	}


	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("IndexerTimedIndexHighNoStop: initialize");
		super.initialize();
		indexer.indexHigh();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
	}

	// Called once after timeout
	@Override
	public void end(boolean interrupted) {
		System.out.println("IndexerTimedIndexHighNoStop: end");
		//indexer.stop();
		super.end(interrupted);
	}
}
