package frc.robot.auton.trajectories;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.auton.AutonConstants;
import frc.robot.commands.drivetrain.*;
import frc.robot.subsystems.*;

// moves forward by specified distance
public class MoveLeftOfFuelReef extends SequentialCommandGroup {

	public MoveLeftOfFuelReef(CommandSwerveDrivetrain drivetrain, RobotContainer container) {
		
		addCommands(
			new DrivetrainSwerveRelative(drivetrain, container, createMoveLeftOfFuelReefTrajectory(container))
		); 
  
	}
	
	public Trajectory createMoveLeftOfFuelReefTrajectory(RobotContainer container) {
		// An example trajectory to follow. All units in meters.
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Pass through these waypoints
			List.of(),
			// End straight ahead of where we started, facing forward
			// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
			new Pose2d(0, -AutonConstants.ONE_THIRD_OF_A_METER, Rotation2d.fromDegrees(0)),
			container.createTrajectoryConfig());

		return trajectory;
	}

}