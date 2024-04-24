// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

import frc.robot.commands.IntakeCmd;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class NoteRstCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_indexer;
  private final Intake m_intake;

  private boolean x;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public NoteRstCmd(Indexer m_indexer, Intake m_intake) {
    this.m_indexer = m_indexer;
    this.m_intake = m_intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_indexer);
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { //sets indexer speed and starts the timer also stops intake
    m_intake.setIntake(0);
    m_indexer.startIndexTimer();
    m_indexer.setIndexer(-Constants.indexBackOutSpeed);
    m_intake.setIntake(-Constants.indexBackOutSpeed);

    x= false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { //runs until timer above threshould then indexer off and x is true so command enters isFinished
    //m_indexer.startIndexTimer();
    if(m_indexer.getIndexTimer() >= Constants.indexBackOutTime){
      x = true;
      m_indexer.setIndexer(0);
      m_intake.setIntake(0);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.setIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    return x;
  }
}
