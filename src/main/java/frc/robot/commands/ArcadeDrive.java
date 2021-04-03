// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;
  private final Supplier<Boolean> m_lTankTurnSupplier;
  private final Supplier<Boolean> m_rTankTurnSupplier;


  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param zaxisRotateSuppplier Lambda supplier of rotational speed
   */
  public ArcadeDrive(
      Drivetrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSuppplier,
      Supplier<Boolean> lTankTurnSupplier,
      Supplier<Boolean> rTankTurnSupplier) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSuppplier;
    m_lTankTurnSupplier = lTankTurnSupplier;
    m_rTankTurnSupplier = rTankTurnSupplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_xaxisSpeedSupplier.get();
    x = x*0.82;
    double z = m_zaxisRotateSupplier.get();
    if (Math.abs(z) < 0.1) z = 0;
    if (Math.abs(x) < 0.1) x = 0;
    z = z*0.7;
    boolean lturn = m_lTankTurnSupplier.get(), rturn = m_rTankTurnSupplier.get();
    if (lturn || rturn) {
      double turnSpeed = 0.6;
      double rot = (lturn ? -turnSpeed : 0) + (rturn ? turnSpeed : 0);
      m_drivetrain.arcadeDrive(0.8, rot);
    }
    else {
      m_drivetrain.arcadeDrive(x, z);
    }
    SmartDashboard.putNumber("Joystick Foward", x);
    SmartDashboard.putNumber("Joystick Rotate", z);

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
