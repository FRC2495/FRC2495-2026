package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Feeder;

/**
 * Add your docs here.
 */
public class FeederTimedFeedHighNoStop extends WaitCommand {
	/**
	 * Add your docs here.
	 */
	private Feeder feeder;

	public FeederTimedFeedHighNoStop(Feeder feeder, double timeout) {
		super(timeout);
		this.feeder = feeder;
		addRequirements(feeder);
	}


	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("FeederTimedFeedHighNoStop: initialize");
		super.initialize();
		feeder.feedHigh();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
	}

	// Called once after timeout
	@Override
	public void end(boolean interrupted) {
		System.out.println("FeederTimedFeedHighNoStop: end");
		//feeder.stop();
		super.end(interrupted);
	}
}
