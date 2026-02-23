package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Indexer;

/**
 *
 */
public class IndexerStopForever extends Command {

	private Indexer indexer;

	public IndexerStopForever(Indexer indexer) {
		this.indexer = indexer;
		addRequirements(indexer);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("IndexerStopForever: initialize");
		indexer.stop();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		// nothing
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false; // we run forever (unless interrupted)
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("IndexerStopForever: end");
	}
}
