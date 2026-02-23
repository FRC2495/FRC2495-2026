package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 *
 */
public class IndexerJoystickControl extends Command {

	private Indexer indexer;
	//private SwerveDrivetrain drivetrain;
	private Joystick joystick;


	public IndexerJoystickControl(Indexer indexer, CommandSwerveDrivetrain drivetrain, Joystick joystick) {
		this.indexer = indexer;
		//this.drivetrain = drivetrain;
		this.joystick = joystick;
		
		addRequirements(
			indexer,
			drivetrain); // this is needed so that the default drivetrain command does not run at the same time
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("IndexerJoystickControl: initialize");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		indexer.joystickControl(joystick);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("IndexerJoystickControl: end");
		indexer.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	/*@Override
	public void interrupted() {
		System.out.println("IndexerJoystickControl: interrupted");
		end(); // TODO check if this is a good idea
	}*/
}
