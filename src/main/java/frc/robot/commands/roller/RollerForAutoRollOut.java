
package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Roller;

/**
 *
 */
public class RollerForAutoRollOut extends Command {

	private Roller roller;

	public RollerForAutoRollOut(Roller roller) {
		this.roller = roller;
		addRequirements(roller);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("RollerForAutoRollOut: initialize");
		roller.rollOut();
	}

	@Override
	public boolean isFinished() {
		return !roller.isFuelExitingAuto();
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("RollerForAutoRollOut: end");
		//roller.stop();
	}

}
