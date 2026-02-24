package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Feeder;

/**
 *
 */
public class FeederStopForever extends Command {

	private Feeder feeder;

	public FeederStopForever(Feeder feeder) {
		this.feeder = feeder;
		addRequirements(feeder);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("FeederStopForever: initialize");
		feeder.stop();
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
		System.out.println("FeederStopForever: end");
	}
}
