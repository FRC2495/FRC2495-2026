
package frc.robot.commands.coral_roller;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Roller;

/**
 *
 */
public class RollerStopForever extends Command {

	private Roller coral_roller;

	public RollerStopForever(Roller coral_roller) {
		this.coral_roller = coral_roller;
		addRequirements(coral_roller);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("RollerStopForever: initialize");
		coral_roller.stop();
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
		System.out.println("RollerStopForever: end");
	}
}
