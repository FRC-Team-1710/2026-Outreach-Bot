// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.Shooter_State;

public class Intake extends SubsystemBase {

	private Shooter_State m_state;

	public Intake() {
		this.m_state = Shooter_State.IDLE;
	}


	public void periodic()
	{
		switch (this.m_state) {
			case IDLE: {
				break;
			}

			case RUNNING: {
				break;
			}
		}
	}


	public Shooter_State getState()
	{
		return this.m_state;
	}

	public void setState(Shooter_State state)
	{
		this.m_state = state;
	}
}
