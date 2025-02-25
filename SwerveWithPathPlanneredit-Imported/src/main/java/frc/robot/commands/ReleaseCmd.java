// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ReleaseCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final Indexer m_indexer;
  //private final Shooter m_shooter;
  private final XboxController m_controller = new XboxController(0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ReleaseCmd(Indexer m_indexer) {
    this.m_indexer = m_indexer;
    //this.m_shooter = m_shooter;
    //m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_indexer);
    //addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //if(m_shooter.getShooterRPM() > Constants.shooterRPMLim){
      m_indexer.setIndexer(Constants.indexerPush);
      //m_shooter.runShooter(Constants.shooterSpeed);
    //}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.setIndexer(0);
    //m_shooter.runShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if(m_shooter.getShooterRPM() > Constants.shooterRPMLim){

      if(m_controller.getLeftTriggerAxis() > 0.5){
         return true;
      }
      else{
        return false;
      }
    //}
    //else{
      //return true;
    }}
  //}
//}
