package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Feeder;

public class FeederDecreasePresetRpm extends InstantCommand {

	private Feeder feeder;

	public FeederDecreasePresetRpm(Feeder feeder) {
		this.feeder = feeder;
		addRequirements(feeder);
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		//System.out.println("FeederDecreasePresetRpm: initialize");
		feeder.decreasePresetRps();
	}
}
