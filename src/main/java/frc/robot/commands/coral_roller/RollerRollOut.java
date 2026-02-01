
package frc.robot.commands.coral_roller;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Roller;

/**
 *
 */
public class RollerRollOut extends Command {

	private Roller coral_roller;

	public RollerRollOut(Roller coral_roller) {
		this.coral_roller = coral_roller;
		addRequirements(coral_roller);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("RollerRollOut: initialize");
		coral_roller.rollOut();
	}

}
