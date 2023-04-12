// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveTrain;
import frc.robot.commands.StopAutoCommand;

public class chasis extends SubsystemBase {
  /** Creates a new chasis. */
  private TalonFX frontLeftMotor;
  private TalonFX frontRightMotor;
  private TalonFX backLeftMotor;
  private TalonFX backRightMotor;
  public double[] startPoints = new double[4];
  public double[] motorPosition = new double[4];
  private final DifferentialDriveOdometry m_odometry;
  private AHRS navx = new AHRS(SerialPort.Port.kMXP);
  Pose2d pose;
  double timeStart = 0;

  public chasis() {
    // Declares the four driving motors as a variable
    frontLeftMotor = new TalonFX(Constants.FRONT_LEFT_ID);
    frontRightMotor = new TalonFX(Constants.FRONT_RIGHT_ID);
    backLeftMotor = new TalonFX(Constants.BACK_LEFT_ID);
    backRightMotor = new TalonFX(Constants.BACK_RIGHT_ID);

    // frontLeftMotor
    // frontRightMotor
    // backLeftMotor
    // backRightMotor

    frontLeftMotor.setNeutralMode(NeutralMode.Coast);
    frontRightMotor.setNeutralMode(NeutralMode.Coast);
    backLeftMotor.setNeutralMode(NeutralMode.Coast);
    backRightMotor.setNeutralMode(NeutralMode.Coast);
    // Left motors are inverted so this corrects it
    frontLeftMotor.setInverted(true);
    backLeftMotor.setInverted(true);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(
        this.getGyroRotation(), this.ticksToMeters(frontLeftMotor.getSelectedSensorPosition()),
        this.ticksToMeters(frontRightMotor.getSelectedSensorPosition()));
    // FlyWheelMotor.configFactoryDefault();
    // FlyWheelMotor.configNeutralDeadband(0.001);
    // FlyWheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
    // Constants.kPIDLoopIdx, Constants.kTimeoutsMs);
    // FlyWheelMotor.configNominalOutputForward(0, Constants.kTimeoutsMs);
    // FlyWheelMotor.configNominalOutputReverse(0, Constants.kTimeoutsMs);
    // FlyWheelMotor.configPeakOutputForward(1, Constants.kTimeoutsMs);
    // FlyWheelMotor.configPeakOutputReverse(-1, Constants.kTimeoutsMs);
    // FlyWheelMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF,
    // Constants.kTimeoutsMs);
    // FlyWheelMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP,
    // Constants.kTimeoutsMs);
    // FlyWheelMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI,
    // Constants.kTimeoutsMs);
    // FlyWheelMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD,
    // Constants.kTimeoutsMs);

  }

  public void initializeMotors() {
    initializeMotor(frontLeftMotor, 0);
    initializeMotor(frontRightMotor, 1);
    initializeMotor(backLeftMotor, 2);
    initializeMotor(backRightMotor, 3);
    timeStart = System.currentTimeMillis();

  }

  private void initializeMotor(TalonFX motor, int index) {
    motor.configFactoryDefault();
    motor.configNeutralDeadband(0.001);
    // motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
    // 30);
    // motor.configNominalOutputForward(0, Constants.kTimeoutsMs);
    // motor.configNominalOutputReverse(0, Constants.kTimeoutsMs);
    // motor.configPeakOutputForward(1, Constants.kTimeoutsMs);
    // motor.configPeakOutputReverse(-1, Constants.kTimeoutsMs);
    // motor.config_kF(0, 0, Constants.kTimeoutsMs);
    // motor.config_kP(0, 1, Constants.kTimeoutsMs);
    // motor.config_kI(0, 0, Constants.kTimeoutsMs);
    // motor.config_kD(0, 0, Constants.kTimeoutsMs);

    startPoints[index] = motor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    double tempSpeed = getWheelSpeeds().leftMetersPerSecond;
    m_odometry.update(
        this.getGyroRotation(), this.ticksToMeters(frontLeftMotor.getSelectedSensorPosition()),
        this.ticksToMeters(frontRightMotor.getSelectedSensorPosition()));
    SmartDashboard.putNumber("Encoder", frontLeftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("x distance moved", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("y distance moved", m_odometry.getPoseMeters().getY());
    setDefaultCommand(new DriveTrain());
    // This method will be called once per scheduler run
  }

  // This moves the left motors
  public void moveLeft(double speed) {
      frontLeftMotor.set(ControlMode.PercentOutput, speed);
      backLeftMotor.set(ControlMode.PercentOutput, speed);
  }

  // This moves the right motors
  public void moveRight(double speed) {
    frontRightMotor.set(ControlMode.PercentOutput, speed);
    backRightMotor.set(ControlMode.PercentOutput, speed);
  }

  public double[] getMotorPosition() {
    motorPosition[0] = frontLeftMotor.getSelectedSensorPosition();
    motorPosition[1] = frontRightMotor.getSelectedSensorPosition();
    motorPosition[2] = backLeftMotor.getSelectedSensorPosition();
    motorPosition[3] = backRightMotor.getSelectedSensorPosition();
    return motorPosition;
  }

  public void updateStartPoints() {
    startPoints[0] = frontLeftMotor.getSelectedSensorPosition();
    startPoints[1] = frontRightMotor.getSelectedSensorPosition();
    startPoints[2] = backLeftMotor.getSelectedSensorPosition();
    startPoints[3] = backRightMotor.getSelectedSensorPosition();
  }

  // this was added last night but is not being called.
  // we need t oset up a push button to toggle between coast mode and
  // brake mode.
  public void SetCoastMode() {
    frontLeftMotor.setNeutralMode(NeutralMode.Coast);
    frontRightMotor.setNeutralMode(NeutralMode.Coast);
    backLeftMotor.setNeutralMode(NeutralMode.Coast);
    backRightMotor.setNeutralMode(NeutralMode.Coast);

  }

  public void SetBrakeMode() {
    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    backLeftMotor.setNeutralMode(NeutralMode.Brake);
    backRightMotor.setNeutralMode(NeutralMode.Brake);
  }

  public Pose2d getPose() {
    // pose = m_odometry.update(this.getGyroRotation(),
    // frontLeftMotor.getSelectedSensorPosition() / 1000,
    // frontRightMotor.getSelectedSensorPosition() / 1000);
    // SmartDashboard.putNumber("PoseX", pose.getX());
    // SmartDashboard.putNumber("PoseY", pose.getY());
    // return pose;
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        this.getGyroRotation(), this.ticksToMeters(frontLeftMotor.getSelectedSensorPosition()),
        this.ticksToMeters(frontRightMotor.getSelectedSensorPosition()),
        pose);
  }

  public void resetEncoders() {
    frontLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    SmartDashboard.putNumber("Left Vel", this.ticksToMeters(frontLeftMotor.getSelectedSensorVelocity()));
    SmartDashboard.putNumber("Right Vel", this.ticksToMeters(frontRightMotor.getSelectedSensorVelocity()));
    return new DifferentialDriveWheelSpeeds(this.ticksToMeters(frontLeftMotor.getSelectedSensorVelocity()),
        this.ticksToMeters(frontRightMotor.getSelectedSensorVelocity()));
  }

  public Rotation2d getGyroRotation() {
    SmartDashboard.putNumber("yaw", navx.getYaw());
    return Rotation2d.fromDegrees(navx.getYaw());
    // return Rotation2d.fromDegrees(30);
  }

  public double getYaw() {
    return navx.getYaw();
  }

  public double getPitch() {
    return navx.getPitch();
  }

  public double getAngle() {
    return navx.getAngle();
  }

  public Command followTrajectoryCommand(
  // PathPlannerTrajectory traj, boolean isFirstPath
  ) {
    return new SequentialCommandGroup(
        // new InstantCommand(() -> {
        // // Reset odometry for the first path you run during auto
        // if (isFirstPath) {
        // this.resetOdometry(traj.getInitialPose());
        // }
        // }),
        // new PPRamseteCommand(
        // traj,
        // this::getPose, // Pose supplier
        // new RamseteController(),
        // new SimpleMotorFeedforward(Constants.ksVolts,
        // Constants.kvVoltSecondsPerMeter,
        // Constants.kaVoltSecondsSquaredPerMeter),
        // this.kDriveKinematics, // DifferentialDriveKinematics
        // this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
        // new PIDController(0.00005, 0, 0), // Left controller. Tune these values for
        // your robot. Leaving them 0 will
        // // only
        // // use feedforwards.
        // new PIDController(0.00005, 0, 0), // Right controller (usually the same
        // values as left controller)
        // this::tankDriveVolts, // Voltage biconsumer
        // true, // Should the path be automatically mirrored depending on alliance
        // color.
        // // Optional, defaults to true
        // this // Requires this drive subsystem
        // ),
        new StopAutoCommand());
  }

  public double ticksToMeters(double ticks) {
    return (ticks * ((6 * Math.PI) / 21504)) / 39.37;
  }
}
