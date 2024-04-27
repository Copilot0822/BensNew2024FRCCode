// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BackPhotonVision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** An example command that uses an example subsystem. */
public class AutoAim extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final Arm m_arm;
  private final BackPhotonVision m_photon;
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAim(Arm m_arm, BackPhotonVision m_photon, CommandSwerveDrivetrain drivetrain) {
    //m_subsystem = subsystem;
    this.m_arm = m_arm;
    this.m_photon = m_photon;
    //this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
    addRequirements(m_arm);
    addRequirements(m_photon);
    addRequirements(drivetrain);

    //final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      
      //.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
    double x = m_photon.getX();
    double y = m_photon.getY();
    //final SwerveRequest.RobotCentric drive = new SwerveRequest(). 
    drivetrain.setControl(drive.withRotationalRate(y*3));
    
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return !m_photon.hasTargets();
    return false;
  }
}