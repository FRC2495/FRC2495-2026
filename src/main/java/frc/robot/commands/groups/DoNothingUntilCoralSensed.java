
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DoNothing;
import frc.robot.sensors.CoralSensor;
import frc.robot.sensors.NoteSensor;
import frc.robot.subsystems.Roller;

/**
 *
 */
public class DoNothingUntilCoralSensed extends Command {

	private Roller roller;
	private CoralSensor coral_sensor;


	public DoNothingUntilCoralSensed(Roller roller) {
		this.roller = roller;
		//this.coral_sensor = coral_sensor;
		addRequirements(roller);
	}


	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("DoNothingUntilCoralSensed: initialize");
	}

	@Override
	public boolean isFinished() {
		return roller.hasCoral();
		//return !notesensor.isEnergized() || !noteSensorTwo.isEnergized();
	}

	@Override
	public void end(boolean interupted) {
		System.out.println("DoNothingUntilCoralSensed: end");
	}

}
