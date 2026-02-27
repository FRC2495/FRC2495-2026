
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Neck;

import frc.robot.commands.roller.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.indexer.*;
import frc.robot.commands.feeder.*;
import frc.robot.commands.neck.*;

/**
 * A command group that stops all the subsystems that are used in the shooting process, except for the drivetrain and hanger.
 * This is used to stop all the subsystems at once when the driver presses the back or start button on the copilot gamepad.
 */
public class AlmostEverythingStop extends SequentialCommandGroup {

	public AlmostEverythingStop(Roller roller, Shooter shooter, Indexer indexer, Feeder feeder, Neck neck) {

		addCommands(
			new NeckStop(neck),
			new ShooterStop(shooter),
			new IndexerStop(indexer),
			new FeederStop(feeder),
			new RollerStop(roller));
	} 
}