// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO.Indexer_State;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO.Intake_State;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO.Shooter_State;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
	private final CommandXboxController m_Controller;

	private final Intake m_intake;
	private final Indexer m_indexer;
	private final Shooter m_shooter;

	private final DriveSubsystem m_drive;

	public RobotContainer() {
		/*
		CommandXboxController scannerController = null;
		while(scannerController == null) {
			for (int i = 0; i < 5; i++) {
				scannerController = new CommandXboxController(i);

				if (scannerController.isConnected())
				{
					break;
				}
			}

			if (scannerController == null) {
				System.out.println("NO CONTROLLER CONNECTED!");
			}
		}

		this.m_Controller = scannerController;
		*/

		this.m_Controller = new CommandXboxController(0);

		this.m_drive = new DriveSubsystem();
		
		this.m_intake = new Intake();
		this.m_indexer = new Indexer();
		this.m_shooter = new Shooter();
		
		configureBindings();
	}
	
	private void configureBindings() {
		this.m_drive.setDefaultCommand(Commands.run(() -> this.m_drive.drive(new Translation2d(-this.m_Controller.getLeftY(), -this.m_Controller.getLeftX()), -this.m_Controller.getRightX(), true), this.m_drive));

		this.m_Controller.start().onTrue(Commands.runOnce(() -> this.m_drive.resetGyroscope()));

		switch (Constants.Drivers.Shoot_Mode) {
			case ONE_BUTTON_SHOOT: {
				break;
			}
		
			case TWO_BUTTON_SHOOT:{
				this.m_Controller.rightTrigger().whileTrue(new InstantCommand(() -> this.m_shooter.setState(Shooter_State.REV), this.m_shooter));
				this.m_Controller.rightTrigger().whileTrue(new InstantCommand(() -> this.m_shooter.setHoodHeight(this.m_Controller.getRightTriggerAxis()), this.m_shooter));

				this.m_Controller.rightBumper().whileTrue(new InstantCommand(() -> this.m_intake.setState(Intake_State.INTAKING)));

				this.m_Controller.a().onTrue(new InstantCommand(() -> this.m_indexer.setState(Indexer_State.INDEXING), this.m_indexer));

				break;
			}
		}
	}


	public Command getAutonomousCommand() {
		return null;
	}
}
