
package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Roller;

/**
 *
 */
public class RollerRollOut extends Command {

	private Roller roller;

	public RollerRollOut(Roller roller) {
		this.roller = roller;
		addRequirements(roller);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("RollerRollOut: initialize");
		roller.rollOut();
	}

}
