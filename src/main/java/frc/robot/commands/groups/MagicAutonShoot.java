
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.PhotonVisionSystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Neck;

import frc.robot.commands.roller.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.indexer.*;
import frc.robot.commands.feeder.*;
import frc.robot.commands.neck.*;

import frc.robot.vision.PhotonVisionSystem;

/**
 * A command group that can be used to shoot in autonomous.
 * 
 * It is called "MagicAutonShoot" because it is a combination of all the commands that are needed to shoot in autonomous.
 */
public class MagicAutonShoot extends SequentialCommandGroup {

	public MagicAutonShoot(Roller roller, Shooter shooter, Indexer indexer, Feeder feeder, Neck neck, PhotonVisionSystem vision) {

		addCommands(
			new ShooterShootUsingCamera(shooter, vision), // starts the shooter at the correct RPM based on the distance to the hub
			new WaitCommand(3), // waits for a few seconds

			new IndexerIndexHigh(indexer), // starts the indexer to index the fuel into the shooter
			new FeederFeedHigh(feeder), // feeds the fuel into the indexer

			new WaitCommand(2), // waits for a few seconds

			new FeederStop(feeder), // stops the feeder
			new IndexerStop(indexer), // stops the indexer
			new ShooterStop(shooter) // stops the shooter	
		);
	} 
}