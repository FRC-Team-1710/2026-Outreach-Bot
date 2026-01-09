// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.Indexer_State;

public class Indexer extends SubsystemBase {

	private Indexer_State m_state;

	public Indexer() {
		this.m_state = Indexer_State.IDLE;
	}


	public void periodic()
	{
		switch (this.m_state) {
			case IDLE: {
				break;
			}

			case INATKING: {

				break;
			}

			case INDEXING: {

				break;
			}
		}
	}


	public Indexer_State getState()
	{
		return this.m_state;
	}

	public void setState(Indexer_State state)
	{
		if (this.m_state == Indexer_State.INDEXING) return;
		
		this.m_state = state;
	}
}
