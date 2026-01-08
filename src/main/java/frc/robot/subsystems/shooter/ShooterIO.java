package frc.robot.subsystems.shooter;

public interface ShooterIO {
	
	public enum Shooter_State
	{
		IDLE,
		RUNNING
	}

	class ShooterIOInputs {
		public Shooter_State state;
	}

	public void updateInputs(ShooterIOInputs input);
}