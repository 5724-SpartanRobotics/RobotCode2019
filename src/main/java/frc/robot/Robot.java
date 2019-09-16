/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
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

  //private DiagnosticsLogger Diagnostics;

  private XboxController xbox;
  private Joystick joystick;

  //private DoubleSolenoid ClimbFront;
  //private DoubleSolenoid ClimbBack;
  private VictorSP BackFootMover;
  private Spark FrontFootMover;
  private SpeedControllerGroup FeetMovers;
  
  // Fun fact: on the 2019 robot, 1 turn of the drivetrain wheels is approximately
  // 4.831113433837891 encoder "rotations" (which should be 4.831113433837891 rotations of the motors)
  // Calculated 21.85mph left side and 21.32mph right side
  private CANSparkMax MainLeft;
  private CANSparkMax AltLeft;
  private CANSparkMax MainRight;
  private CANSparkMax AltRight;
  private SRamp SpeedRamp;

  private DifferentialDrive Drive;
  
  private TalonSRX Lifter;
  private TalonSRX LiftFollower;

  private VictorSPX LegFrontL;
  private VictorSPX LegFrontR;
  private VictorSPX LegBackL;
  private Spark LegBackR;

  private int LiftSetpoint;
  
  private Compressor Comp;

  private Solenoid ArmOpener;
  private Solenoid ArmExtender;
  private Spark ArmGrippers;
  private boolean ArmsExtended = false;
  private boolean ArmsClosed = false;

  private DigitalInput HatchSwitch0;
  private DigitalInput HatchSwitch1;

  // Hook #2 Values
  //public static final int HATCH_BOTTOM = -3602;
  //public static final int HATCH_MIDDLE = -13490;
  //public static final int HATCH_TOP = -22085;
  // Hook #1 Values
  public static final int HATCH_BOTTOM = -3516;//-4805;
  public static final int HATCH_MIDDLE = -13120;
  public static final int HATCH_TOP = -21214;//-23320;
  public static final int CARGO_PICKUP = 0;
  public static final int CARGO_BOTTOM = -7610;
  public static final int CARGO_MIDDLE = -16026;//-17686;
  public static final int CARGO_TOP = -24458;//-24834;
  public static final int CARGO_FLOOR = 0;
  // We actually program this into the motor controller
  public static final int LIMIT_UP = -26000;//-27500;

  private SRamp LiftRamp;
  
  public NetworkTable VisionTable;
  public NetworkTableEntry TapeDetectedEntry;
  public NetworkTableEntry TapePitchEntry;
  public NetworkTableEntry TapeYawEntry;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // LED stuff
  public NetworkTable LedTable;
  public NetworkTableEntry LedScript;
  public NetworkTableEntry LedScriptArgument;

  // SAFETY MODE
  public static final boolean SAFETY_MODE = false;
  private boolean safetyTripped = false;
  private static final int MAX_SAFETY_COUNT = 50;
  private int safetyCount = MAX_SAFETY_COUNT;

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

    Comp = new Compressor();

    //ClimbBack = new DoubleSolenoid(PCM_COMP_24V, 0, 1);
    //ClimbFront = new DoubleSolenoid(PCM_COMP_24V, 2, 3);
    LegFrontL = new VictorSPX(30);
    LegFrontR = new VictorSPX(31);
    LegBackL = new VictorSPX(32);
    LegBackR = new Spark(9);

    BackFootMover = new VictorSP(1);
    FrontFootMover = new Spark(2);
    //FeetMovers = new SpeedControllerGroup(BackFootMoverFootMover);

    MainRight = new CANSparkMax(9, MotorType.kBrushless);
    AltRight = new CANSparkMax(10, MotorType.kBrushless);
    MainLeft = new CANSparkMax(11, MotorType.kBrushless);
    AltLeft = new CANSparkMax(12, MotorType.kBrushless);

    MainRight.setIdleMode(IdleMode.kCoast);
    AltRight.setIdleMode(IdleMode.kCoast);
    MainLeft.setIdleMode(IdleMode.kCoast);
    AltLeft.setIdleMode(IdleMode.kCoast);

    AltLeft.follow(MainLeft);
    AltRight.follow(MainRight);

    Drive = new DifferentialDrive(MainLeft, MainRight);

    Lifter = new TalonSRX(6);
    Lifter.setNeutralMode(NeutralMode.Brake);
    Lifter.enableCurrentLimit(false);
    /*Lifter.configContinuousCurrentLimit(40);
    Lifter.configPeakCurrentLimit(50);
    Lifter.configPeakCurrentDuration(1500);*/
    //Lifter.configReverseSoftLimitEnable(true);
    //Lifter.configReverseSoftLimitThreshold(-27000);
    //Lifter.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    //Lifter.configClearPositionOnLimitF(true, 0);
    Lifter.selectProfileSlot(0, 0);
    LiftSetpoint = 0;

    LiftFollower = new TalonSRX(5);
    LiftFollower.follow(Lifter);
    LiftFollower.setNeutralMode(NeutralMode.Brake);

    ArmExtender = new Solenoid(0);
    ArmOpener = new Solenoid(1);
    ArmGrippers = new Spark(0);

    HatchSwitch0 = new DigitalInput(0);
    HatchSwitch1 = new DigitalInput(1);

    //Diagnostics = new DiagnosticsLogger();
    // Uncomment this line to enable diagnostics. Warning: this may
    // cause the robot to be slower than it is supposed to be because of lag.
    //Diagnostics.start();

    xbox = new XboxController(0);
    joystick = new Joystick(1);
    
    LiftRamp = new SRamp();
    SpeedRamp = new SRamp();

    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    VisionTable = nt.getTable("ChickenVision");
    TapeDetectedEntry = VisionTable.getEntry("tapeDetected");
    TapePitchEntry = VisionTable.getEntry("tapePitch");
    TapeYawEntry = VisionTable.getEntry("tapeYaw");

    LedTable = nt.getTable("LedInfo");
    LedScript = LedTable.getEntry("CurrentScript");
    LedScriptArgument = LedTable.getEntry("ScriptArgument");

    UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
    cam.setPixelFormat(PixelFormat.kMJPEG);
    cam.setResolution(320, 240);
    cam.setFPS(20);
    

    LedScript.setString("ColorWaves");
    LedScriptArgument.setString("");
  }

  private static final String[][] LED_SCRIPTS = { //{"ChristmasCombo1"}, 
                                                  {"Binary", "{\"msg\":\"We are spartans 5724\"}"}, {"ChristmasCombo1"},
                                                  {"ColorWaves"}, {"MultiColorScript"}, //{"ChristmasCombo1"},
                                                  {"RandomChristmas"}, {"ChristmasCombo1"},
                                                  {"MultiColorScript"}, {"ColorWaves"}, //{"ChristmasCombo1"},
                                                  {"MorseCode", "We are spartans 5724"}, {"ChristmasCombo1"} };
  private int count = 0;
  private final int SCRIPT_LENGTH = 6000;

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
    /*count++;

    if (count < 0) {
      count = 0;
    }

    int scriptIdx = count / SCRIPT_LENGTH;

    String[] script = LED_SCRIPTS[scriptIdx % (LED_SCRIPTS.length)];

    LedScript.setString(script[0]);
    
    if (script.length > 1) {
      LedScriptArgument.setString(script[1]);
    }*/

    // Hatch 0 has to be inverted because it is normally closed
    //SmartDashboard.putBoolean("HatchSwitch0", !HatchSwitch0.get());
    //SmartDashboard.putBoolean("HatchSwitch1", HatchSwitch1.get());


    final int MAX_TEMP = 100;
    if (MainLeft.getMotorTemperature() >= MAX_TEMP || AltLeft.getMotorTemperature() >= MAX_TEMP) {
      xbox.setRumble(RumbleType.kLeftRumble, 0.5);
    } else {
      xbox.setRumble(RumbleType.kLeftRumble, 0);
    }
    
    if (MainRight.getMotorTemperature() >= MAX_TEMP || AltRight.getMotorTemperature() >= MAX_TEMP) {
      xbox.setRumble(RumbleType.kRightRumble, 0.5);
    } else {
      xbox.setRumble(RumbleType.kRightRumble, 0);
    }

    //SmartDashboard.putNumber("DaTa", NetworkTableInstance.getDefault().getTable("VisionTable").getEntry("THIS IS A RANDOM NUMBER").getNumber(-1).doubleValue());
    //SmartDashboard.putNumber("LiftSetpoint", LiftSetpoint);
    //SmartDashboard.putNumber("LiftEncoderPos", Lifter.getSelectedSensorPosition());
    //SmartDashboard.putNumber("LiftVoltageOut", Lifter.getMotorOutputVoltage());
    //SmartDashboard.putNumber("LiftCurrentOut", Lifter.getOutputCurrent());
    //SmartDashboard.putString("ControlMode", Lifter.getControlMode().toString());
    //SmartDashboard.putNumber("ClosedLoopError", Lifter.getClosedLoopError());
    //SmartDashboard.putNumber("LiftRampOutput", liftRamp.getOutput());

    /*Diagnostics.writeDouble("DrivePosLeft", MainLeft.getEncoder().getPosition());
    Diagnostics.writeDouble("DrivePosRight", MainRight.getEncoder().getPosition());
    Diagnostics.writeDouble("DrivePosLeftAlt", AltLeft.getEncoder().getPosition());
    Diagnostics.writeDouble("DrivePosRightAlt", AltRight.getEncoder().getPosition());
    Diagnostics.writeDouble("LiftVoltageOut", Lifter.getMotorOutputVoltage());
    Diagnostics.writeDouble("LiftCurrentOut", Lifter.getOutputCurrent());
    Diagnostics.writeInteger("LiftEncoderPos", Lifter.getSelectedSensorPosition());
    Diagnostics.writeInteger("LiftSetpoint", LiftSetpoint);
    Diagnostics.writeInteger("LiftRampOutput", liftRamp.getOutput());
    
    Diagnostics.timestamp();*/
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    //Diagnostics.writeString("State", "DISABLED");
  }

  @Override
  public void disabledPeriodic() {
    //Scheduler.getInstance().run();
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
    //Diagnostics.writeString("State", "AUTO");
    //m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    /*if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }*/

    teleopInit();
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
    //Diagnostics.writeString("State", "TELEOP");

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    safetyCount = 0;

    clearAllButtonStates();

    ArmsExtended = ArmsClosed = false;
    //ClimbFront.set(Value.kReverse);
    //ClimbBack.set(Value.kReverse);

    LiftSetpoint = Lifter.getSelectedSensorPosition();
    LiftRamp = new SRamp();
    LiftRamp.Rate = 300;
    LiftRamp.setOutput(Lifter.getSelectedSensorPosition());

    SpeedRamp = new SRamp();
    SpeedRamp.Rate = 0.06;
    SpeedRamp.setMaxAccelRate(0.004);

    VisionTable.getEntry("Tape").setBoolean(true);

    Comp.start();
  }

  // This checks all the buttons at teleop enable so that
  // the check for pressed since last check in teleopPeriodic
  // does not check between disables.
  private void clearAllButtonStates() {
    for (int i = 1; i <= 10; i++) {
      xbox.getRawButtonPressed(i);
    }

    for (int i = 1; i <= 12; i++) {
      joystick.getRawButtonPressed(i);
    }
  }

  boolean joyPOV0PressedLast = false;
  boolean joyPOV180PressedLast = false;
  boolean IsClimbingFront = false;
  boolean IsClimbingBack = false;

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double y = -xbox.getRawAxis(0) * 0.7D * (1.0D + Math.max(0, xbox.getRawAxis(4)) * 1.4285714D);
    double rawSpeed = xbox.getRawAxis(2) - xbox.getRawAxis(3);

    if (SAFETY_MODE) {
      y *= 0.75D;

      if (xbox.getBumper(Hand.kLeft) && joystick.getRawButton(7)) {
        if (safetyCount > 0) {
          safetyCount--;
          SmartDashboard.putNumber("Safety Count", safetyCount);

          if (safetyCount == 0) {
            safetyTripped = false;
            clearAllButtonStates();
            SmartDashboard.putBoolean("Safety Tripped", safetyTripped);
          }
        } else {
          safetyTripped = false;
        }
      } else {
        safetyTripped = true;
        safetyCount = MAX_SAFETY_COUNT;
        SmartDashboard.putNumber("Safety Count", safetyCount);
        SmartDashboard.putBoolean("Safety Tripped", safetyTripped);
      }
    } else {
      safetyTripped = false;
    }

    // Climb Speed
    if (xbox.getRawButton(6)) {
      rawSpeed = -0.3D;
    }

    // Super slow mode
    if (xbox.getAButton() || SAFETY_MODE) {
      SpeedRamp.Setpoint = rawSpeed * 0.45D;
    } else {
      SpeedRamp.Setpoint = rawSpeed * 0.8D * (1.0D + Math.max(0, xbox.getRawAxis(4)) * 0.25);
    }

    SpeedRamp.update();

    boolean seesTape = TapeDetectedEntry.getBoolean(false);
    
    if (seesTape && !safetyTripped) {
      final double ADJUST_CONST = 0.13281734;
      //double tapePitch = TapePitchEntry.getNumber(0).doubleValue();
      double tapeYaw = TapeYawEntry.getNumber(0).doubleValue();
      final double TARGET_YAW = 0;//PitchYawAdjuster.GetYawFromPitch(tapePitch);

      //SmartDashboard.putNumber("tapePitch", tapePitch);
      //SmartDashboard.putNumber("tapeYaw", tapeYaw);
      
      double diff = tapeYaw - ADJUST_CONST;
      
      String s = "";

      if (diff > 0) {
        s = "<-- (" + diff + ")";
      } else if (diff < 0) {
        s = "--> (" + diff + ")";
      }

      if (/*xbox.getRawButton(5) This is now taken for the safety button*/false) {
        final double MAX_AFFECT = 0.4;
        if (diff > MAX_AFFECT) {
          diff = MAX_AFFECT;
        } else if (diff < -MAX_AFFECT) {
          diff = -MAX_AFFECT;
        }

        y -= diff;
      }

      SmartDashboard.putString("TapeDir", s);
    } else {
      SmartDashboard.putString("TapeDir", "X");
    }

    if (xbox.getRawButtonPressed(7)) {
      Lifter.setSelectedSensorPosition(0);
      LiftSetpoint = 0;
      LiftRamp.Setpoint = 0;
      LiftRamp.setOutput(0);
    }

    //Scheduler.getInstance().run();
    // Cargo ship is(n't anymore) -13120

    // The fine adjustment has nothing to do with hammers.
    // Don't try to use a hammer on the roboRIO. Ever.
    final int FINE_ADJUSTMENT_AMOUNT = -500;

    if (!safetyTripped) {
      if (joystick.getPOV() == 0) {
        if (!joyPOV0PressedLast) {
          joyPOV0PressedLast = true;
          if (LiftSetpoint - FINE_ADJUSTMENT_AMOUNT <= 0)
            LiftSetpoint -= FINE_ADJUSTMENT_AMOUNT;
        }
      } else {
        joyPOV0PressedLast = false;

        if (joystick.getPOV() == 180) {
          if (!joyPOV180PressedLast) {
            joyPOV180PressedLast = true;
            LiftSetpoint += FINE_ADJUSTMENT_AMOUNT;
          }
        } else {
          joyPOV180PressedLast = false;
        }
      }

      Drive.arcadeDrive(SpeedRamp.getOutput(), y);

      if (joystick.getRawButtonPressed(11)) {
        LiftSetpoint = HATCH_BOTTOM;
      } else if (joystick.getRawButtonPressed(9)) {
        LiftSetpoint = HATCH_MIDDLE;
      } else if (joystick.getRawButtonPressed(7)) {
        // This button is now used for safety mode LiftSetpoint = HATCH_TOP;
      } else if (joystick.getRawButtonPressed(12)) {
        LiftSetpoint = CARGO_BOTTOM;
      } else if (joystick.getRawButtonPressed(10)) {
        LiftSetpoint = CARGO_MIDDLE;
      } else if (joystick.getRawButtonPressed(8)) {
        //LiftSetpoint = CARGO_TOP;
      } else if (joystick.getRawButtonPressed(1)) {
        LiftSetpoint = CARGO_FLOOR;
      }

      double liftY = -joystick.getRawAxis(1);
      final double deadband = 0.15;

      if (Math.abs(liftY) > deadband) {
        double change = (liftY < 0 ? liftY + 0.15 : liftY - 0.15) * 200;//(int)liftEntry.getDouble(0);//
        
        if (change > 100) {
          change = 100;
        } else if (change < -100) {
          change = -100;
        }

        LiftRamp.setOutput(LiftRamp.getOutput() + change);
        LiftRamp.Setpoint += change;
        LiftSetpoint = (int)LiftRamp.Setpoint;
      }

      if (LiftSetpoint < LIMIT_UP) {
        LiftSetpoint = LIMIT_UP;
      }
    } else {
      Drive.arcadeDrive(0, 0);
    }
    
    LiftRamp.Setpoint = LiftSetpoint;
    SmartDashboard.putNumber("LiftSetpoint", LiftRamp.Setpoint);
    SmartDashboard.putNumber("LiftSetpoint", Lifter.getSelectedSensorPosition());
    SmartDashboard.putNumber("LiftOutput", LiftRamp.getOutput());
    LiftRamp.update();

    Lifter.set(ControlMode.Position, LiftRamp.getOutput());
    //Lifter.set(ControlMode.PercentOutput, -joystick.getRawAxis(1));

    if (!safetyTripped) {
      if (joystick.getRawButtonPressed(4)) {
        ArmsClosed = !ArmsClosed;
      } else if (!SAFETY_MODE) {
        // This is the same as the joystick safety button, so it only works outside
        // safety mode, and why would it be needed in safety mode anyway?
        // It should be disabled in safety mode anyway to make sure we don't
        // accidently attack someone's fingers at a Demo.

        // Holding joystick button 7 makes the limit switches on the hatch mechanism
        // active, so when the hatch contacts the switches, the mechanism automatically
        // grabs the hatch.
        // HatchSwitch0 does not have to be inverted. It's complicated.
        if (joystick.getRawButton(7) && (HatchSwitch0.get() || !HatchSwitch1.get())) {
          ArmsClosed = false;
        }
      }

      if (joystick.getRawButtonPressed(3)) {
        ArmsExtended = !ArmsExtended;
    }
    }

    ArmExtender.set(ArmsExtended);
    ArmOpener.set(ArmsClosed);

    //Diagnostics.writeDouble("DriveX", x);
    //Diagnostics.writeDouble("DriveY", y);

    // X = out, Y = in

    final double GRAB_SPEED;

    if (SAFETY_MODE) {
      GRAB_SPEED = 0.8D;
    } else {
      GRAB_SPEED = 1D; // TODO at one point we had this 0.8, is that what it's supposed to be?
    }
    
    if (!safetyTripped) {
      ArmGrippers.set(/*xbox.getXButton() || */joystick.getRawButton(6) ? -GRAB_SPEED : /*xbox.getYButton() ||*/ joystick.getRawButton(5) ? GRAB_SPEED : 0);
    } else {
      ArmGrippers.set(0);
    }


    // === Climbing stuff ===
    boolean climbSafety = joystick.getRawButton(2) && !safetyTripped;
    
    // Have to check all of these every update to make sure it was pressed
    // between now and the last update
    boolean climbFront = xbox.getYButton/*Pressed*/();
    boolean climbBack = xbox.getXButton/*Pressed*/();
    boolean climbBoth = xbox.getRawButton/*Pressed*/(8);

    if (climbSafety && (climbFront || climbBack)) {
      final double CLIMB_SPEED = 1;
      if (climbFront) {
        LegFrontR.set(ControlMode.PercentOutput, CLIMB_SPEED);
        LegFrontL.set(ControlMode.PercentOutput, -CLIMB_SPEED);
        LegBackR.set(CLIMB_SPEED);
        LegBackL.set(ControlMode.PercentOutput, -CLIMB_SPEED);
        IsClimbingFront = !IsClimbingFront;
      }
      if (climbBack) {
        final double RETRACT_SPEED = 0.75;
        LegFrontR.set(ControlMode.PercentOutput, -RETRACT_SPEED);
        LegFrontL.set(ControlMode.PercentOutput, RETRACT_SPEED);
        LegBackR.set(-RETRACT_SPEED);
        LegBackL.set(ControlMode.PercentOutput, RETRACT_SPEED);
        IsClimbingBack = !IsClimbingBack;
      }
      if (climbBoth) {
        // Set them all to the opposite of where the front
        // currently is.
        IsClimbingFront = IsClimbingBack = !IsClimbingFront;
      }
    } else {
      LegFrontR.set(ControlMode.PercentOutput, 0);
      LegFrontL.set(ControlMode.PercentOutput, 0);
      LegBackR.set(0);
      LegBackL.set(ControlMode.PercentOutput, 0);
    }
    //FrontFootMover.set(1);//Math.max(-1.0, Math.min(5 * -SpeedRamp.getOutput(), 1.0)));
    //BackFootMover.set(Math.max(-1.0, Math.min(5 * -SpeedRamp.getOutput(), 1.0)));

    double footSpeed = Math.max(-1.0, Math.min(rawSpeed * 3, 1.0F));
/*
    if (IsClimbingFront && !safetyTripped) {
      //ClimbFront.set(DoubleSolenoid.Value.kForward);
      FrontFootMover.set(footSpeed);
    } else {
      //ClimbFront.set(DoubleSolenoid.Value.kReverse);
      LegFrontR.set(ControlMode.PercentOutput, 0);
      LegFrontL.set(ControlMode.PercentOutput, 0);
      FrontFootMover.set(0);
    }

    if (IsClimbingBack && !safetyTripped) {
      //ClimbBack.set(DoubleSolenoid.Value.kForward);
      BackFootMover.set(footSpeed);
    } else {
      //ClimbBack.set(DoubleSolenoid.Value.kReverse);
      BackFootMover.set(0);
    }

    /*if (climbState == 0) {
    } else if (climbState == 1) {
      ClimbFront.set(DoubleSolenoid.Value.kForward);
      ClimbBack.set(DoubleSolenoid.Value.kForward);
      FeetMovers.set(Math.max(-1.0, Math.min(5 * SpeedRamp.getOutput(), 1.0)));
    } else {
      ClimbFront.set(DoubleSolenoid.Value.kReverse);
      ClimbBack.set(DoubleSolenoid.Value.kForward);
      FeetMovers.set(Math.max(-1.0, Math.min(5 * SpeedRamp.getOutput(), 1.0)));
    }*/

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    //Diagnostics.writeString("State", "TEST");
  }
}
