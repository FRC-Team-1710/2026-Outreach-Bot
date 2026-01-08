// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO.Shooter_State;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
	private final CommandXboxController m_Controller;

	private final Shooter m_shooter;
	private final DriveSubsystem m_drive;

	public RobotContainer() {
		this.m_Controller = new CommandXboxController(0);
		
		this.m_drive = new DriveSubsystem();
		this.m_shooter = new Shooter();
		
		configureBindings();
	}
	
	private void configureBindings() {
		this.m_drive.setDefaultCommand(Commands.run(() -> this.m_drive.drive(new Translation2d(-this.m_Controller.getLeftY(), -this.m_Controller.getLeftX()), -this.m_Controller.getRightX(), true), this.m_drive));

		this.m_Controller.leftBumper().onTrue(Commands.run(() -> this.m_shooter.setState(Shooter_State.RUNNING), this.m_shooter));
		this.m_Controller.start().onTrue(Commands.runOnce(() -> this.m_drive.resetGyroscope()));
	}


	public Command getAutonomousCommand() {
		return null;
	}
}
