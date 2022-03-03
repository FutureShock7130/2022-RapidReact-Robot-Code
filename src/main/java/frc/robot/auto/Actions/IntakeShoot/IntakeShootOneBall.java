// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.Actions.IntakeShoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.TimedIntake;
import frc.robot.commands.Transporter.TimedTransport;
import frc.robot.commands.Turret.TimedTurret;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transporter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeShootOneBall extends ParallelCommandGroup {
  /** Creates a new IntakeShoot. */
  public IntakeShootOneBall(Intake m_robotIntake, Transporter m_roboTransporter, Turret m_robotTurret, double autoShootSpeedPercentageOutput, double intakeTimeSec, double transportTimeSec, double shootTimeSec) {
  
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new TimedTurret(m_robotTurret, shootTimeSec, autoShootSpeedPercentageOutput),
        new SequentialCommandGroup(
          new TimedIntake(intakeTimeSec, m_robotIntake),
          new WaitCommand(0.2),
          new TimedTransport(intakeTimeSec, m_roboTransporter)
        )
      )
    );
  }
}
