// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClawToggle;
import frc.robot.commands.DriveOutAuto;
import frc.robot.commands.DriveOutLeft2Piece;
import frc.robot.commands.DriveOutRight2Piece;
import frc.robot.commands.EncoderMoveForward;
import frc.robot.commands.EncoderRotate;
import frc.robot.commands.LeftAuto;
import frc.robot.commands.RightAuto;
import frc.robot.commands.StopAutoCommand;
import frc.robot.commands.ManualModeToggle;
import frc.robot.commands.MiddleAuto;
import frc.robot.commands.MiddleAutoV2;
import frc.robot.commands.NavX;
import frc.robot.commands.PathPlannerLeft;
import frc.robot.commands.limelightCommand;
import frc.robot.subsystems.arm;
import frc.robot.subsystems.chasis;
import frc.robot.subsystems.clawSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static chasis m_chasis = new chasis();
  public static arm m_arm = new arm();
  public static clawSubsystem m_claw = new clawSubsystem();
  private Joystick Leftjoy = new Joystick(Constants.LEFT_JOYSTICK_ID);
  private Joystick Rightjoy = new Joystick(Constants.RIGHT_JOYSTICK_ID);
  private XboxController Controller1 = new XboxController(Constants.XBOXCONTROLLER_ID);
  private CommandXboxController ccontroller = new CommandXboxController(Constants.XBOXCONTROLLER_ID);
  public JoystickButton button12 = new JoystickButton(Leftjoy, Constants.BUTTON_12_ID);
  public JoystickButton button10 = new JoystickButton(Leftjoy, Constants.BUTTON_10_ID);
  public JoystickButton button9 = new JoystickButton(Leftjoy, Constants.BUTTON_9_ID);
  public JoystickButton button8 = new JoystickButton(Leftjoy, Constants.BUTTON_8_ID);
  public JoystickButton button7 = new JoystickButton(Leftjoy, Constants.BUTTON_7_ID);
  public JoystickButton lTrigger = new JoystickButton(Leftjoy, Constants.BUTTON_1_ID);
  public JoystickButton rTrigger = new JoystickButton(Rightjoy, Constants.BUTTON_1_ID);
  public JoystickButton button2 = new JoystickButton(Rightjoy, Constants.BUTTON_2_ID);
  Trigger xButton = ccontroller.x(); // Creates a new Trigger object for the `X` button on exampleCommandController
  Trigger yButton = ccontroller.y();
  boolean abutton = Controller1.getAButtonPressed();
  Trigger lbumper = ccontroller.leftBumper();
  Trigger rbumper = ccontroller.rightBumper();
  Trigger XboxButton9 = ccontroller.button(9);
  Trigger XboxButton10 = ccontroller.button(10);
  Trigger backButton = ccontroller.back();
  private Command m_right_auto = new RightAuto();
  // private Command m_middle_auto = new MiddleAuto();
  private Command m_middle_auto = new MiddleAutoV2();
  private Command m_left_auto = new LeftAuto();
  private Command m_driveOut = new DriveOutAuto();
  private Command m_testauto = new MiddleAuto();
  private Command m_drive_out_left = new DriveOutLeft2Piece();
  private Command m_drive_out_right = new DriveOutRight2Piece();

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public double offset;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    m_chooser.addOption("Drive Out", m_driveOut);
    m_chooser.addOption("Left Position", m_left_auto);
    m_chooser.addOption("Middle Position", m_middle_auto);
    // m_chooser.addOption("Middle Position", m_middle_auto);
    m_chooser.addOption("Right Position", m_right_auto);
    m_chooser.addOption("Middle Mobility", m_testauto);
    // m_chooser.addOption("Drive Out Left 2 Piece", m_drive_out_left);
    // m_chooser.addOption("Drive Out right 2 Piece", m_drive_out_right);
    // m_chooser.addOption("PathPlanner", new SequentialCommandGroup(new PathPlannerLeft(), new StopAutoCommand()));
    //PathPlannerTrajectory examplePath = PathPlanner.loadPath("Simple Path", new PathConstraints(4, 3));
    //m_chooser.addOption("PathPlanner", m_chasis.followTrajectoryCommand(examplePath, false));
    SmartDashboard.putData("Auton Selector", m_chooser);
    configureBindings();
    turnOffLED();
    offset = getPitch();
  }

  private void configureBindings() {
    // When you press the button it will call the command and it will keep running
    // until the robot balances
    button12.whileTrue(new NavX(true));
    button8.whileTrue(new NavX(false));
    // Runs limemlightCommand while button 7 is pressed
    button7.whileTrue(new limelightCommand());
    button9.onTrue(new EncoderMoveForward(50, 0.3, 1000));
    lTrigger.onTrue(new EncoderRotate(-90, 0.3, 2500));
    rTrigger.onTrue(new EncoderRotate(90, Constants.SPIN_SPEED, 2500));
    button2.onTrue(new EncoderRotate(180, Constants.SPIN_SPEED, 2500));
    backButton.onTrue(new ManualModeToggle());
    rbumper.onTrue(new ClawToggle());
  }

  public double getJoyStickTriggerL() {
    return Controller1.getLeftTriggerAxis();
  }

  public double getJoyStickTriggerR() {
    return Controller1.getRightTriggerAxis();
  }

  public boolean getdpad() {
    return Controller1.getPOV() >= 0;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  // Variables that stores the joysticks
  public boolean getXboxButton9() {
    return XboxButton9.getAsBoolean();
  }

  public boolean getXboxButton10() {
    return XboxButton10.getAsBoolean();
  }

  // joystick axises
  public double GetleftjoyRawAxis(int laxis) {
    return Leftjoy.getRawAxis(laxis);
  }

  public double GetRightjoyRawAxis(int raxis) {
    return Rightjoy.getRawAxis(raxis);
  }

  public double GetXboxStickLeft() {
    return Controller1.getLeftY();
  }

  public double GetXboxStickRight() {
    return Controller1.getRightY();
  }

  // This represents the Navx as a variable
  private AHRS navx = new AHRS(SerialPort.Port.kMXP);

  // NavX Gyroscope Methods that get values //
  public void calibrateGyro() {
    navx.calibrate();
  }

  public double isArmButtonPressed() {
    SmartDashboard.putBoolean("Start Button", Controller1.getStartButtonPressed());
    SmartDashboard.putBoolean("Y Button", Controller1.getStartButtonPressed());
    if (Controller1.getStartButton())
      // home pos
      return 0.87;
    if (Controller1.getYButton())
      // High pos
      return 0.67;
    if (Controller1.getBButton())
      // Medium pos
      return 0.70;
    if (Controller1.getAButton())
      // Low pos
      return 0.80;
    if (Controller1.getXButton()) {
      // Loading pos
      // return 0.84;
      return 0.71;
    }
    return Math.PI;
  }

  public double isArmButtonPressedBC() {
    if (Controller1.getStartButton())
      // Home
      return Constants.SLED_HOME;
    if (Controller1.getYButton())
      // High
      return Constants.SLED_HIGH;
    if (Controller1.getBButton())
      // Medium
      return Constants.SLED_MEDIUM;
    if (Controller1.getAButton())
      // Low
      return Constants.SLED_LOW;
    if (Controller1.getXButton()) {
      // Loading
      return Constants.SLED_LOADING;
    }
    return Math.PI;
  }

  public boolean isCalibrating() {
    return navx.isCalibrating();
  }

  public boolean isConnected() {
    return navx.isConnected();
  }

  public void zeroGyro() {
    System.out.println("NavX Connected: " + navx.isConnected());
    navx.reset();
  }

  public double getYaw() {
    return RobotContainer.m_chasis.getYaw();
  }

  public double getPitch() {
    return RobotContainer.m_chasis.getPitch();
  }

  public double getRoll() {
    return navx.getRoll();
  }

  public double getAngle() {
    return RobotContainer.m_chasis.getAngle();
  }

  // These are representing the buttons as a variable

  // This stores the limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public double getIsTargetValid() {
    // will return 1 if true 0 if not might change this to a boolean later
    return table.getEntry("tv").getDouble(0.0);
  }

  public double getAngleX() {
    return table.getEntry("tx").getDouble(0.0);
    // shows the horizontal offset from the cross hair between -27 to 27
  }

  public double getAngleY() {
    // Show the vertical offset from the cross hair between -20.5 to 20.5 degrees
    return table.getEntry("ty").getDouble(0.0);
  }

  public double getArea() {
    // This shows a percent of how much space the target is taking up between
    // 0%-100%
    return table.getEntry("ta").getDouble(0.0);
  }

  public void turnOffLED() {
    table.getEntry("ledMode").setNumber(1);
  }

  public void turnOnLED() {
    table.getEntry("ledMode").setNumber(3);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
    // return m_chooser.getSelected();
  }
}
