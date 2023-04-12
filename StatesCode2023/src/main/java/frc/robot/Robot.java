// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.print.attribute.standard.Compression;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  public static RobotContainer m_robotContainer;
  Thread m_visionThread1;

  // UsbCamera camera = new UsbCamera("camerashoot", 0);
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //compressor.enableDigital();
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    Constants.NAVX_OFFSET = Robot.m_robotContainer.getPitch();

    // this is the shooter camera
    m_visionThread1 = new Thread(
        () -> {
          // Get the UsbCamera from CameraServer
          UsbCamera camera = CameraServer.startAutomaticCapture("ArmView", 0);
          // Set the resolution
          camera.setResolution(160, 120);
          camera.setFPS(1);
        });
    m_visionThread1.setDaemon(true);
    m_visionThread1.start();

    /*
     * UsbCamera shootercamera = CameraServer.startAutomaticCapture(0);
     * shootercamera.setResolution(160, 120);
     * shootercamera.setFPS(1);
     */
    // this was the giant block of code that we didn't understand and didn't make
    // sense
    /*
     * UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
     * MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
     * mjpegServer1.setSource(usbCamera);
     * CvSink cvsink1 = new CvSink("opencv_USB Camera 0");
     * cvsink1.setSource(usbCamera);
     * CvSource outputStream = new CvSource("shooter", PixelFormat.kMJPEG, 160, 120,
     * 10);
     * MjpegServer mjpegServer2 = new MjpegServer("serve_shooter", 1182);
     * mjpegServer2.setSource(outputStream);
     */

    // this is the intake camera
    // UsbCamera intakecamera = CameraServer.startAutomaticCapture(1);
    // intakecamera.setResolution(160, 120);
    // intakecamera.setFPS(1);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    Constants.NAVX_OFFSET = Robot.m_robotContainer.getPitch();
    RobotContainer.m_claw.ClawPneuOut();
    RobotContainer.m_arm.auton = true;
    RobotContainer.m_chasis.SetBrakeMode();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    RobotContainer.m_chasis.SetCoastMode();
    RobotContainer.m_arm.auton = false;
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
