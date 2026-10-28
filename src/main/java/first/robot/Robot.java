// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotation;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Volt;

import java.io.Console;
import java.io.PrintStream;
import java.lang.annotation.ElementType;
import java.util.Vector;

import javax.lang.model.util.ElementScanner14;

import org.opencv.core.Mat;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.InstantCommand;
import org.wpilib.framework.TimedRobot;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.LogMessage;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTableValue;
import org.wpilib.units.Unit;
import org.wpilib.units.Units;
import org.wpilib.networktables.NetworkTablesJNI;
import org.wpilib.telemetry.*;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Voltage;
import org.wpilib.vision.camera.CameraServerJNI.TelemetryKind;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.XboxController;
import org.wpilib.system.Notifier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.ffi.CANBusNative;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import first.robot.subsystems.drivetrain.*;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.imu.OnboardIMU;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
*/
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  
  private final RobotContainer robotContainer;
  //private final first.robot.custom.VexController controller = new first.robot.custom.VexController(0);
  
  
  
  XboxController driverController = new XboxController(0);
  XboxController programmerController = new XboxController(1);
  Timer timer2 = new Timer();
  Timer timer = new Timer();
  CANBus CAN_S0 = new CANBus(CANPort.CAN_S0);
  CANBus CAN_S1 = new CANBus(CANPort.CAN_S1);
  TalonFX BackRightSteer = new TalonFX(0, CAN_S0); 
  TalonFX BackRightDrive = new TalonFX(1, CAN_S0);
  TalonFX FrontRightSteer = new TalonFX(3, CAN_S0);
  TalonFX FrontRightDrive = new TalonFX(4, CAN_S0);
  TalonFX BackLeftSteer = new TalonFX(6, CAN_S1);
  TalonFX BackLeftDrive = new TalonFX(7, CAN_S1);
  TalonFX FrontLeftSteer = new TalonFX(9, CAN_S1);
  TalonFX FrontLeftDrive = new TalonFX(10, CAN_S1);
  CANcoder BackRightEncoder = new CANcoder(2, CAN_S0);
  CANcoder FrontRightEncoder = new CANcoder(5, CAN_S0);
  CANcoder BackLeftEncoder = new CANcoder(8, CAN_S1);
  CANcoder FrontLeftEncoder = new CANcoder(11, CAN_S1);
  SparkMax ElevatorActuator = new SparkMax(CANPort.CAN_S0,12, MotorType.kBrushed);
  TalonFX PrimaryElevator = new TalonFX(13, CAN_S1);
  TalonFX SecondaryElevator = new TalonFX(14, CAN_S1);
  SparkMax Blocker = new SparkMax(CANPort.CAN_S1,15, MotorType.kBrushless);

  double[] blockerPos = {-0.45,0,0.35};

  SwerveModule BackRightModule = new SwerveModule(BackRightDrive, BackRightSteer);
  SwerveModule FrontRightModule = new SwerveModule(FrontRightDrive, FrontRightSteer);
  SwerveModule BackLeftModule = new SwerveModule(BackLeftDrive, BackLeftSteer);
  SwerveModule FrontLeftModule = new SwerveModule(FrontLeftDrive, FrontLeftSteer);
  SwerveDrive swerveDrive = new SwerveDrive(21.75, 19.75, FrontLeftModule, FrontRightModule, BackLeftModule, BackRightModule);

  Boolean game = false;
  // NetworkTables control for BackRightModule (voltage-based)
  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final org.wpilib.networktables.NetworkTable brTable = ntInstance.getTable("BackRightModule");
  private final DoubleSubscriber brVoltageSub = brTable.getDoubleTopic("voltage_volts").subscribe(0.0);
  private final DoubleSubscriber brAngleSub = brTable.getDoubleTopic("angle_rad").subscribe(0.0);
  private final DoubleSubscriber brEnableSub = brTable.getDoubleTopic("enable").subscribe(0.0);

  private final DoublePublisher brVoltagePub = brTable.getDoubleTopic("voltage_volts").publish();
  private final DoublePublisher brAnglePub = brTable.getDoubleTopic("angle_rad").publish();
  private final DoublePublisher brEnablePub = brTable.getDoubleTopic("enable").publish();
  private final DoublePublisher brTelemetryAnglePub = brTable.getDoubleTopic("current_angle_rad").publish();
  private final DoublePublisher brTelemetryVoltagePub = brTable.getDoubleTopic("current_voltage_volts").publish();
  private final TalonFXConfiguration steerConfiguration = new TalonFXConfiguration()
  .withMotorOutput(
      new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
  )
  .withClosedLoopGeneral(
      new ClosedLoopGeneralConfigs()
          .withContinuousWrap(true)
  )
  .withCurrentLimits(
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(60))
          .withSupplyCurrentLimit(Amps.of(40))
  )
  .withFeedback(
      new FeedbackConfigs()
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
          .withSensorToMechanismRatio(150.0/7.0)
  )
  .withSlot0(
      new Slot0Configs()
          .withKP(215)
          .withKD(5.35)
          .withKS(0.19)
  );
  
  private final TalonFXConfiguration backRightSteerConfiguration = steerConfiguration.clone();
  private final TalonFXConfiguration frontRightSteerConfiguration = steerConfiguration.clone();
  private final TalonFXConfiguration backLeftSteerConfiguration = steerConfiguration.clone();
  private final TalonFXConfiguration frontLeftSteerConfiguration = steerConfiguration.clone();

  private final TalonFXConfiguration driveConfiguration = new TalonFXConfiguration()
  .withMotorOutput(
      new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
  )
  .withCurrentLimits(
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(80))
          .withSupplyCurrentLimit(Amps.of(40))
  );
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private void setSteerOffsetsFromCanCoders() {
    setSteerOffsetFromCanCoder(BackRightSteer, BackRightEncoder);
    setSteerOffsetFromCanCoder(FrontRightSteer, FrontRightEncoder);
    setSteerOffsetFromCanCoder(BackLeftSteer, BackLeftEncoder);
    setSteerOffsetFromCanCoder(FrontLeftSteer, FrontLeftEncoder);
  }

  private void setSteerOffsetFromCanCoder(TalonFX steerMotor, CANcoder canCoder) {
    Angle absoluteRotations = canCoder.getAbsolutePosition().getValue();
    steerMotor.setPosition(absoluteRotations.times(-1));
  }

  public Robot() {
    timer2.stop();
    BackLeftSteer.getConfigurator().apply(backLeftSteerConfiguration);
    FrontLeftSteer.getConfigurator().apply(frontLeftSteerConfiguration);
    BackRightSteer.getConfigurator().apply(backRightSteerConfiguration);
    FrontRightSteer.getConfigurator().apply(frontRightSteerConfiguration);
    BackLeftDrive.getConfigurator().apply(driveConfiguration);
    FrontLeftDrive.getConfigurator().apply(driveConfiguration);
    BackRightDrive.getConfigurator().apply(driveConfiguration);
    FrontRightDrive.getConfigurator().apply(driveConfiguration);
    SecondaryElevator.setControl(new Follower(PrimaryElevator.getDeviceID(), MotorAlignmentValue.Aligned));
    setSteerOffsetsFromCanCoders();
    initializeResetYawCommand();
    Telemetry.log("Reset Yaw", resetYawCommand);
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    Telemetry.log("Reset", resetCommand);
    robotContainer = new RobotContainer();
    
    // Initialize control values on NetworkTables so UI sees keys immediately
    try {
      brVoltagePub.set(0.0);
      brAnglePub.set(0.0);
      brEnablePub.set(0.0);
      brTelemetryAnglePub.set(BackRightModule.getSteerAngleRadians());
      brTelemetryVoltagePub.set(0.0);
    } catch (Exception e) {
      // ignore NT init errors
    }
    
    // Try to construct the OnboardIMU using the first available MountOrientation enum constant.
    OnboardIMU tempImu = null;
    try {
      Class<?> mountClass = Class.forName("org.wpilib.hardware.imu.OnboardIMU$MountOrientation");
      Object[] constants = mountClass.getEnumConstants();
      if (constants != null && constants.length > 0) {
        java.lang.reflect.Constructor<OnboardIMU> cons = OnboardIMU.class.getConstructor(mountClass);
        tempImu = cons.newInstance(constants[0]);
      }
    } catch (Exception e) {
      tempImu = null;
    }
    imu = tempImu;
  }
  
  private final OnboardIMU imu;
  
  private Command resetYawCommand;
  private boolean lastResetYawButtonState = false;
  
  private void initializeResetYawCommand() {
    resetYawCommand = new InstantCommand(() -> {
      if (imu != null) {
        imu.resetYaw();
      }
    });
  }
  
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and utility.
  *
  * <p>This runs after the mode specific periodic functions, but before LiveWindow and
  * SmartDashboard integrated updating.
  */
 @Override
 public void robotPeriodic() {
   
   // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
   // commands, running already-scheduled commands, removing finished or interrupted commands,
   // and running subsystem periodic() methods.  This must be called from the robot's periodic
   // block in order for anything in the Command-based framework to work.
   CommandScheduler.getInstance().run();
  }
  
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() 
  {
    
  }
  
  @Override
  public void disabledPeriodic() 
  {
  }
  
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  Boolean run = true;

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    timer.restart();
    run = false;
    CommandScheduler.getInstance().schedule(resetYawCommand);
    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }
  
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {
    double yaw = imu.getYawRadians();
//  swerveDrive.globalArcadeDrive(-0.05, 0, 0, Math.PI-yaw);
    swerveDrive.globalArcadeDrive(-0.05, 0, SwerveModule.shortestAngleDelta(yaw,0)*-4, Math.PI-yaw);
    if(timer.get()<5.0)
    {
      ElevatorActuator.setThrottle(1);
    }
    else if(timer.get()<13)
    {
    }
    else if(timer.get()<18)
    {
      ElevatorActuator.setThrottle(-1);
    }
    else if(timer.get()<18.3)
    {
      PrimaryElevator.setControl(new DutyCycleOut(-1));
    }
    else
    {
      PrimaryElevator.setControl(new DutyCycleOut(0));
    }
    timer.get();
  }
  @Override
  public void teleopInit() {
    run = true;
    
    Blocker.getEncoder().setPosition(blockerPos[0]);
    blockerState = 0;
    
    addPeriodic(()->
    {
      double yaw = imu.getYawRadians();
      double multiplier = (1-programmerController.getRightTrigger());
      if(!run) return;
      if(driverController.getRightTrigger() == 0)
      { 
        swerveDrive.globalArcadeDrive(-driverController.getLeftY()*multiplier, driverController.getLeftX()*multiplier, driverController.getRightX()*multiplier,Math.PI-yaw);
      }
      else
      {
        swerveDrive.globalArcadeDrive(-driverController.getLeftY()*multiplier, driverController.getLeftX()*multiplier, 0,Math.PI-yaw);
      }
    }, 0.002);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }
  
  Command resetCommand = new InstantCommand(() -> {
    swerveDrive.resetEncoders();
  });

  
  double t = 0;
  int blockerState = 0;
  Boolean LastPumpState = false;
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {

    if(driverController.getYButton())
    {
      game = true;
    }
    else if(driverController.getXButton())
    {
      game = false;
    }
    // If NetworkTables control is enabled (>0.5), drive the back-right module from the provided VelocityVector values.
    // try {
    //   double enabled = brEnableSub.get();
    //   if (enabled > 0.5) {
    //     double volts = brVoltageSub.get();
    //     double angle = brAngleSub.get();
    //     BackRightModule.set(new VoltageVector(Units.Volts.of(volts), Units.Radians.of(angle)));
    //     // publish telemetry
    //     try {
    //       brTelemetryAnglePub.set(BackRightModule.getSteerAngleRadians());
    //       brTelemetryVoltagePub.set(volts);
    //     } catch (Exception ignored) {}
    //     return;
    //   }
    // } catch (Exception e) {
    //   // Fall back to normal control on any NT errors
    // }

    // Field-centric: use IMU yaw if available, otherwise fall back to robot-centric local drive

    Blocker.getClosedLoopController().setSetpoint(blockerPos[blockerState], ControlType.kPosition, ClosedLoopSlot.kSlot0);

    ElevatorActuator.setThrottle((driverController.getLeftBumperButton() ? 1 : 0) -driverController.getLeftTrigger());

    
    boolean resetYawButtonPressed = driverController.getAButton();
    Telemetry.log("Reset Yaw Button", resetYawButtonPressed);
    if (resetYawButtonPressed && !lastResetYawButtonState) {
      CommandScheduler.getInstance().schedule(resetYawCommand);
    }
    lastResetYawButtonState = resetYawButtonPressed;
    
    if(driverController.getRightBumperButtonPressed())
    {
      blockerState = (blockerState + 1) % 3;
    }

    if(game == true)
    {
      if(driverController.getRightTrigger()<=0.2)
      {
        LastPumpState = false;
      }
      if(driverController.getRightTrigger()>=0.7 && LastPumpState == false)
      {
        timer2.restart();
        LastPumpState = true;
      }
      if(timer2.isRunning() && !timer2.hasElapsed(0.1))
      {
        PrimaryElevator.setControl(new DutyCycleOut(-0.1));
      }
      else if (timer2.isRunning()) 
      {
        PrimaryElevator.setControl(new DutyCycleOut(0.05));
      }
    }
    else
    {
      PrimaryElevator.setControl(new DutyCycleOut(driverController.getRightY()*driverController.getRightTrigger()));
    }
    //BackLeftModule.set(new VoltageVector(Units.Volts.of(Math.hypot(controller.getLeftX(), controller.getLeftY())), Units.Radians.of((Math.PI/2.0)-Math.atan2(controller.getLeftY(),controller.getLeftX()))));
  }

  @Override
  public void utilityInit() {
    // Cancels all running commands at the start of utility mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during utility mode. */
  @Override
  public void utilityPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
