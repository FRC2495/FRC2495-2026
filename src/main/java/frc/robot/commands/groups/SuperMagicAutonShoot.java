
package frc.robot.commands.groups;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.PhotonVisionSystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Neck;

import frc.robot.commands.roller.*;
import frc.robot.commands.shooter.*;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.indexer.*;
import frc.robot.commands.feeder.*;
import frc.robot.commands.neck.*;

import frc.robot.vision.PhotonVisionSystem;

/**
 * A command group that can be used to shoot in autonomous.
 * 
 * It is called "MagicAutonShoot" because it is a combination of all the commands that are needed to shoot in autonomous.
 */
public class SuperMagicAutonShoot extends SequentialCommandGroup {

	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
		.withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

	private final SwerveRequest.FieldCentricFacingAngle targetHub = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(10, 0, 0) //10 kP was 10 but seemed to be too much
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

	public SuperMagicAutonShoot(CommandSwerveDrivetrain drivetrain, Roller roller, Shooter shooter, Indexer indexer, Feeder feeder, Neck neck, PhotonVisionSystem vision) {

		addCommands(

			new ParallelDeadlineGroup(
				new WaitCommand(6), // timeouts entire group after specified time

				drivetrain.applyRequest(()-> {
					if (!vision.isHubTargetValid()) {
						/* Do typical field-centric driving since we don't have a target */
						return drive.withVelocityX(0) // Drive forward with negative Y (forward)
							.withVelocityY(0) // Drive left with negative X (left)
							.withRotationalRate(0); // Drive counterclockwise with negative Z (left)
					} else {
						/* Use the hub target to determine where to aim */
						return targetHub.withTargetDirection(vision.getHeadingToHubFieldRelative())
							.withVelocityX(0) // Drive forward with negative Y (forward)
							.withVelocityY(0); // Drive left with negative X (left)
					}
				}),

				new ShooterShootUsingCamera(shooter, vision), // starts the shooter at the correct RPM based on the distance to the hub
				//new ShooterShootCustom(shooter, 2900), // starts the shooter at a fixed custom RPM

				new SequentialCommandGroup(
				
					new WaitCommand(3), // waits for a few seconds before starting indexer and feeder

					new ParallelDeadlineGroup(
						new WaitCommand(3), // timeouts after specified time

						new IndexerIndexHigh(indexer), // starts the indexer to index the fuel into the shooter
						new FeederFeedHigh(feeder), // feeds the fuel into the indexer

						new SequentialCommandGroup(
							new NeckMoveMidwayWithStallDetection(neck),
							new NeckMoveDownWithStallDetection(neck)
						)
					)
				)
			),

			new FeederStop(feeder), // stops the feeder
			new IndexerStop(indexer), // stops the indexer
			new ShooterStop(shooter), // stops the shooter
			new NeckMoveDownWithStallDetection(neck)
		);
	} 
}