package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 *
 */
public class FeederJoystickControl extends Command {

	private Feeder feeder;
	//private SwerveDrivetrain drivetrain;
	private Joystick joystick;


	public FeederJoystickControl(Feeder feeder, CommandSwerveDrivetrain drivetrain, Joystick joystick) {
		this.feeder = feeder;
		//this.drivetrain = drivetrain;
		this.joystick = joystick;
		
		addRequirements(
			feeder,
			drivetrain); // this is needed so that the default drivetrain command does not run at the same time
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("FeederJoystickControl: initialize");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		feeder.joystickControl(joystick);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("FeederJoystickControl: end");
		feeder.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	/*@Override
	public void interrupted() {
		System.out.println("FeederJoystickControl: interrupted");
		end(); // TODO check if this is a good idea
	}*/
}
