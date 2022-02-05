// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.InfraredSensor;
import edu.wpi.first.wpilibj.Timer;

public class Intake extends CommandBase {
  private BallHandler ballHandler = new BallHandler();
  private InfraredSensor infrared = new InfraredSensor();
  private Timer timer = new Timer();
  private double pivotTime = 500; //milliseconds
  private boolean finished = false;
  /** Creates a new Intake. */
  public Intake(BallHandler b, InfraredSensor ir) {
    ballHandler = b;
    infrared = ir;
    addRequirements(ballHandler, infrared);
    timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() <= pivotTime){
      ballHandler.setPivotPower(-0.1); //motor goes clockwise
    } else {
      ballHandler.setPivotPower(0.0);
      if (!infrared.get()) {
        ballHandler.setFlywheelPower(0.1);
        ballHandler.setRollerPower(-0.1); //roller goes clockwise
      } else {
        ballHandler.setFlywheelPower(0);
        ballHandler.setRollerPower(0);
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
