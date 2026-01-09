// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterIO.Shooter_State;

public class Shooter extends SubsystemBase {
	
	private Shooter_State m_state;

	private final TalonFX m_flyWheel;

	public Shooter() {
		this.m_state = Shooter_State.IDLE;
		this.m_flyWheel = new TalonFX(Constants.MotorIDs.FlyWheelID);
	}


	public void periodic()
	{
		System.out.println(this.m_state);
		switch (this.m_state) {
			case IDLE: {
				this.m_flyWheel.stopMotor();
				break;
			}

			case REV: {
				this.m_flyWheel.set(0.5);
				break;
			}
		}

		this.m_state = Shooter_State.IDLE;
	}


	public Shooter_State getState()
	{
		return this.m_state;
	}
	
	public void setState(Shooter_State state)
	{
		this.m_state = state;
	}


	public void setHoodHeight(double height)
	{

	}
}
