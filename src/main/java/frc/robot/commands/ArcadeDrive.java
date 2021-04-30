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
  private final Supplier<Double> m_speedTauSupplier;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to
   * the speed supplier lambdas. This command does not terminate.
   *
   * @param drivetrain           The drivetrain subsystem on which this command
   *                             will run
   * @param xaxisSpeedSupplier   Lambda supplier of forward/backward speed
   * @param zaxisRotateSuppplier Lambda supplier of rotational speed
   */
  public ArcadeDrive(
      Drivetrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSuppplier,
      Supplier<Boolean> lTankTurnSupplier,
      Supplier<Boolean> rTankTurnSupplier,
      Supplier<Double> speedTau) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSuppplier;
    m_lTankTurnSupplier = lTankTurnSupplier;
    m_rTankTurnSupplier = rTankTurnSupplier;
    m_speedTauSupplier = speedTau;
    addRequirements(drivetrain);
  }

// L Stick sprint button
// Click the stick and it will enter sprint mode, increasing speed.
// Clicking the stick again will revert to normal mode, or by stopping/reversing direction

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double x = m_xaxisSpeedSupplier.get();
    x *= lerp(0.82, 0.82, m_speedTauSupplier.get());
    double z = m_zaxisRotateSupplier.get();
    if (Math.abs(z) < 0.1) z = 0;
    if (Math.abs(x) < 0.1) x = 0;
    z = z*0.6;
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

  public double lerp(double a, double b, double tau) {
    tau = Math.max(Math.min(tau, 1), 0);
    return (1 - tau) * a + tau * b;
  }
}
