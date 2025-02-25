// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PigeonAutoAim;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PigeonZeroAutoAim extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final PigeonAutoAim m_pigeon;
  private boolean x;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PigeonZeroAutoAim(PigeonAutoAim m_pigeon) {
    //m_subsystem = subsystem;
    this.m_pigeon = m_pigeon;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pigeon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x = false;
    m_pigeon.setZero();
    x = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return x;
  }
}
