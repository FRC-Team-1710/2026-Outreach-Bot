package frc.robot.subsystems.indexer;

public interface IndexerIO {

	public enum Indexer_State
	{
		IDLE,
		INATKING,
		INDEXING
	}

	class ShooterIOInputs {
		public Indexer_State state;
	}

	public void updateInputs(ShooterIOInputs input);
}