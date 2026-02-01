
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Roller;
import frc.robot.commands.roller.*;


/**
 *
 */
public class AlmostEverythingStop extends SequentialCommandGroup {

	public AlmostEverythingStop(/*Elevator elevator,*/ /*Neck neck,*/ Roller roller/* /* , AlgaeRoller algae_roller*/) {

		addCommands(
			//new ElevatorStop(elevator),
			//new DrawerStop(drawer),
			//new NeckStop(neck),
			new RollerStop(roller));
	} 
}