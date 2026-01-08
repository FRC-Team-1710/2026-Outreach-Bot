package frc.robot.subsystems.intake;

public interface IntakeIO {
	
	public enum Intake_State
	{
		IDLE,
		INTAKING,
		OUTTAKING
	}

	class ShooterIOInputs {
		public Intake_State state;
	}

	public void updateInputs(ShooterIOInputs input);
}