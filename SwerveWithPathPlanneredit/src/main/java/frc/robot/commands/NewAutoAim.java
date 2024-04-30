// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BackPhotonVision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PigeonAutoAim;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;

/** An example command that uses an example subsystem. */
public class NewAutoAim extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private final BackPhotonVision m_photon;
  private final PigeonAutoAim m_pigeonAutoAim;
  private final XboxController m_controller = new XboxController(0);
  private double  MaxAngularRate = 1.5 * Math.PI;
  private boolean g;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public NewAutoAim(CommandSwerveDrivetrain m_drivetrain, BackPhotonVision m_photon, PigeonAutoAim m_pigeonAutoAim) {
    //this.m_drivetrain = m_drivetrain;
    this.m_photon = m_photon;
    this.m_pigeonAutoAim = m_pigeonAutoAim;

    //m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    addRequirements(m_photon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1)//.withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

    Rotation2d rotation2d = new Rotation2d( Units.degreesToRadians(m_pigeonAutoAim.getRotation()));
    g = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1*(0.25))//.withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric


    if(g = false){
      Rotation2d rotation2d = new Rotation2d( Units.degreesToRadians(m_pigeonAutoAim.getRotation()));
      Pose2d newPose = new Pose2d(null, rotation2d);
      m_drivetrain.seedFieldRelative(newPose);
      g = true;
    }
    
      // driving in open loop
      
    double x = m_photon.getX();
    double y = m_photon.getY();
    if(y != 0){
      m_drivetrain.setControl(drive.withRotationalRate(y*8));
    } 
    else if(m_controller.getRightX() > 0.1 || m_controller.getRightX() < -0.1){
      m_drivetrain.setControl(drive.withRotationalRate(-m_controller.getRightX() * MaxAngularRate));
      
    }
    else{
      m_drivetrain.setControl(drive.withRotationalRate(0));
    }
    m_drivetrain.setControl(drive.withVelocityX(m_controller.getLeftX() * MaxSpeed*0.25));
    m_drivetrain.setControl(drive.withVelocityY(-m_controller.getLeftY()* MaxSpeed*0.25));
    //m_controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
