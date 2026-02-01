
package frc.robot.commands.coral_roller;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Roller;

/**
 *
 */
public class RollerStop extends InstantCommand {

	private Roller coral_roller;

	public RollerStop(Roller coral_roller) {
		this.coral_roller = coral_roller;
		addRequirements(coral_roller);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("RollerStop: initialize");
		coral_roller.stop();
	
	}

}
