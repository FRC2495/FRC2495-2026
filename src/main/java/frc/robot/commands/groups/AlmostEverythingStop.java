
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.CoralRoller;
import frc.robot.commands.coral_roller.*;


/**
 *
 */
public class AlmostEverythingStop extends SequentialCommandGroup {

	public AlmostEverythingStop(/*Elevator elevator,*/ /*Neck neck,*/ CoralRoller coral_roller/* /* , AlgaeRoller algae_roller*/) {

		addCommands(
			//new ElevatorStop(elevator),
			//new DrawerStop(drawer),
			//new NeckStop(neck),
			new CoralRollerStop(coral_roller));
	} 
}