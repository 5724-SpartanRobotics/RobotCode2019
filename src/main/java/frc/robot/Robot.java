/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;

  private DiagnosticsLogger Diagnostics;

  private XboxController xbox;
  private Joystick joystick;

  private DoubleSolenoid ClimbFront;
  private DoubleSolenoid ClimbBack;
  
  private CANSparkMax MainLeft;
  private CANSparkMax AltLeft;
  private CANSparkMax MainRight;
  private CANSparkMax AltRight;

  private DifferentialDrive Drive;
  
  private TalonSRX Lifter;

  private int LiftSetpoint;
  
  private Compressor Comp;

  private Solenoid ArmOpener;
  private Solenoid ArmExtender;
  private Spark ArmGrippers;

  public static final int PCM_COMP_24V = 1;
  public static final int PCM_12V = 2;

  public static final int HATCH_BOTTOM = 0;
  public static final int HATCH_MIDDLE = 0;
  public static final int HATCH_TOP = 0;
  public static final int CARGO_PICKUP = 0;
  public static final int CARGO_BOTTOM = 0;
  public static final int CARGO_MIDDLE = 0;
  public static final int CARGO_TOP = 0;
  //public static final int LIMIT_UP = -27000;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

    //ClimbFront = new DoubleSolenoid(PCM_COMP_24V, 0, 1);
    //ClimbBack = new DoubleSolenoid(PCM_COMP_24V, 2, 3);
    Comp = new Compressor(PCM_COMP_24V);

    MainRight = new CANSparkMax(9, MotorType.kBrushless);
    AltRight = new CANSparkMax(10, MotorType.kBrushless);
    MainLeft = new CANSparkMax(11, MotorType.kBrushless);
    AltLeft = new CANSparkMax(12, MotorType.kBrushless);

    AltLeft.follow(MainLeft);
    AltRight.follow(MainRight);

    Drive = new DifferentialDrive(MainLeft, MainRight);

    Lifter = new TalonSRX(60);
    
    //Lifter.selectProfileSlot(0, 0);
    LiftSetpoint = 0;

    ArmExtender = new Solenoid(PCM_12V, 0);
    ArmOpener = new Solenoid(PCM_12V, 1);
    ArmGrippers = new Spark(0);

    Diagnostics = new DiagnosticsLogger();
    // Uncomment this line to enable diagnostics. Warning: this may
    // cause the robot to be slower than it is supposed to be because of lag.
    //Diagnostics.start();

    xbox = new XboxController(0);
    joystick = new Joystick(1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //SmartDashboard.putNumber("DaTa", NetworkTableInstance.getDefault().getTable("VisionTable").getEntry("THIS IS A RANDOM NUMBER").getNumber(-1).doubleValue());
    SmartDashboard.putNumber("LiftSetpoint", LiftSetpoint);
    SmartDashboard.putNumber("LiftEncoderPos", Lifter.getSelectedSensorPosition());
    SmartDashboard.putNumber("LiftVoltageOut", Lifter.getMotorOutputVoltage());
    SmartDashboard.putNumber("LiftCurrentOut", Lifter.getOutputCurrent());
    SmartDashboard.putString("ControlMode", Lifter.getControlMode().toString());
    SmartDashboard.putNumber("ClosedLoopError", Lifter.getClosedLoopError());

    Diagnostics.writeDouble("DrivePosLeft", MainLeft.getEncoder().getPosition());
    Diagnostics.writeDouble("DrivePosRight", MainRight.getEncoder().getPosition());
    Diagnostics.writeDouble("DrivePosLeftAlt", AltLeft.getEncoder().getPosition());
    Diagnostics.writeDouble("DrivePosRightAlt", AltRight.getEncoder().getPosition());
    Diagnostics.writeInteger("LiftEncoderPos", Lifter.getSelectedSensorPosition());
    Diagnostics.writeInteger("LiftSetpoint", LiftSetpoint);
    
    Diagnostics.timestamp();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    Diagnostics.writeString("State", "DISABLED");
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    Diagnostics.writeString("State", "AUTO");
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
    //Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    Diagnostics.writeString("State", "TELEOP");

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    Comp.start();
  }

  boolean wasPressed = false;
  int climbState = 0;
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    double y = -xbox.getRawAxis(0);
    double x = (xbox.getRawAxis(2) - xbox.getRawAxis(3)) * 0.8D;

    Drive.arcadeDrive(x, y);

    if (joystick.getRawButton(11)) {
      LiftSetpoint = HATCH_BOTTOM;
    } else if (joystick.getRawButton(9)) {
      LiftSetpoint = HATCH_MIDDLE;
    } else if (joystick.getRawButton(7)) {
      LiftSetpoint = HATCH_TOP;
    } else if (joystick.getRawButton(10)) {
      LiftSetpoint = CARGO_BOTTOM;
    } else if (joystick.getRawButton(8)) {
      LiftSetpoint = CARGO_MIDDLE;
    } else if (joystick.getRawButton(6)) {
      LiftSetpoint = CARGO_TOP;
    }
    
    LiftSetpoint += Math.round(controller.getRawAxis(4) * 20) * 20;
    //Lifter.overrideLimitSwitchesEnable
    // TODO add software upper limit for lift

    //Lifter.set(ControlMode.Position, LiftSetpoint);
    Lifter.set(ControlMode.PercentOutput, xbox.getRawAxis(4));

    ArmExtender.set(xbox.getAButton());
    ArmOpener.set(xbox.getBButton());

    Diagnostics.writeDouble("DriveX", x);
    Diagnostics.writeDouble("DriveY", x);

    ArmGrippers.set(xbox.getYButton() ? -1D : xbox.getXButton() ? 1D : 0);
    
    if (RobotController.getUserButton()) {
      if (!wasPressed) {
        wasPressed = true;
        climbState++;
        climbState %= 3;
      }
    } else {
      wasPressed = false;
    }

    if (1 > 0) return;

    if (climbState == 0) {
      ClimbFront.set(DoubleSolenoid.Value.kReverse);
      ClimbBack.set(DoubleSolenoid.Value.kReverse);
    } else if (climbState == 1) {
      ClimbFront.set(DoubleSolenoid.Value.kForward);
      ClimbBack.set(DoubleSolenoid.Value.kForward);
    } else {
      ClimbFront.set(DoubleSolenoid.Value.kReverse);
      ClimbBack.set(DoubleSolenoid.Value.kForward);
    }

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Diagnostics.writeString("State", "TEST");
  }
}
