/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.motorcontrol.can.*;

// the following were added to support Spark MAX controllers (17 JAN 2020 - PG)

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  // constant added to support choice of Neo Brushless Motors with Spark Max
  // or Falcon Brushless with built-in Talon Fx

  private static final boolean USE_FALCON = false;
  private static final boolean USE_NEO  = ! USE_FALCON;

  // constants added to support Spark MAX controllers

  private static final int LEFT_NEO_LEADER_ID = 4;
  private static final int LEFT_NEO_FOLLOWER1_ID = 5;
  private static final int LEFT_NEO_FOLLOWER2_ID = 6;

  private static final int RIGHT_NEO_LEADER_ID = 7;
  private static final int RIGHT_NEO_FOLLOWER1_ID = 8;
  private static final int RIGHT_NEO_FOLLOWER2_ID = 9;

  // constants added to support Falcon FX (intrisic to Falcon motors) controllers
  
  private static final int LEFT_FALCON_LEADER_ID = 20;
  private static final int LEFT_FALCON_FOLLOWER1_ID = 21;
  private static final int LEFT_FALCON_FOLLOWER2_ID = 22;

  private static final int RIGHT_FALCON_LEADER_ID = 23;
  private static final int RIGHT_FALCON_FOLLOWER1_ID = 24;
  private static final int RIGHT_FALCON_FOLLOWER2_ID = 25;
  
  // Choose controller
  private static DifferentialDrive m_robotDrive;

  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();

  // define Talon FX (intrinsic to Falcon motors) controllers here (24 Jan 2020 PG)

  private WPI_TalonFX m_leftLeadFalconMotor;
  private WPI_TalonFX m_leftFollowerFalconMotor1;
  private WPI_TalonFX m_leftFollowerFalconMotor2;
  
  private WPI_TalonFX m_rightLeadFalconMotor;
  private WPI_TalonFX m_rightFollowerFalconMotor1;
  private WPI_TalonFX m_rightFollowerFalconMotor2;

  // define Spark MAX controllers here (17 JAN 2020 PG)

  private CANSparkMax m_leftLeadNeoMotor;
  private CANSparkMax m_leftFollowerNeoMotor1;
  private CANSparkMax m_leftFollowerNeoMotor2;

  private CANSparkMax m_rightLeadNeoMotor;
  private CANSparkMax m_rightFollowerNeoMotor1;
  private CANSparkMax m_rightFollowerNeoMotor2;

  // sample code recommends collecting left-side and right-side drive motors into speed controller groups
  // (see https://docs.wpilib.org/en/latest/docs/software/actuators/wpi-drive-classes.html) (17 JAN 2020 PG)
    
  SpeedControllerGroup m_left;
  SpeedControllerGroup m_right;
    
  DifferentialDrive m_drive;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    // Select motor types, NEO or Falcon

    if (USE_NEO) {

      // define motors

      m_leftLeadNeoMotor = new CANSparkMax(LEFT_NEO_LEADER_ID, MotorType.kBrushless);
      m_leftFollowerNeoMotor1 = new CANSparkMax(LEFT_NEO_FOLLOWER1_ID, MotorType.kBrushless);
      m_leftFollowerNeoMotor2 = new CANSparkMax(LEFT_NEO_FOLLOWER2_ID, MotorType.kBrushless);

      m_rightLeadNeoMotor = new CANSparkMax(RIGHT_NEO_LEADER_ID, MotorType.kBrushless);
      m_rightFollowerNeoMotor1 = new CANSparkMax(RIGHT_NEO_FOLLOWER1_ID, MotorType.kBrushless);
      m_rightFollowerNeoMotor2 = new CANSparkMax(RIGHT_NEO_FOLLOWER2_ID, MotorType.kBrushless);

      m_left = new SpeedControllerGroup(m_leftLeadNeoMotor,m_leftFollowerNeoMotor1,m_leftFollowerNeoMotor2);
      m_right = new SpeedControllerGroup(m_rightLeadNeoMotor,m_rightFollowerNeoMotor1,m_rightFollowerNeoMotor2);

    } // end if (USE_NEO)

    else if (USE_FALCON) {

      // define motors

      m_leftLeadFalconMotor = new WPI_TalonFX(LEFT_FALCON_LEADER_ID);
      m_leftFollowerFalconMotor1 = new WPI_TalonFX(LEFT_FALCON_FOLLOWER1_ID);
      m_leftFollowerFalconMotor2 = new WPI_TalonFX(LEFT_FALCON_FOLLOWER2_ID);

      m_rightLeadFalconMotor = new WPI_TalonFX(RIGHT_FALCON_LEADER_ID);
      m_rightFollowerFalconMotor1 = new WPI_TalonFX(RIGHT_FALCON_FOLLOWER1_ID);
      m_rightFollowerFalconMotor2 = new WPI_TalonFX(RIGHT_FALCON_FOLLOWER2_ID);

      m_left = new SpeedControllerGroup(m_leftLeadFalconMotor, m_leftFollowerFalconMotor1, m_leftFollowerFalconMotor2);
      m_right = new SpeedControllerGroup(m_rightLeadFalconMotor, m_rightFollowerFalconMotor1, m_rightFollowerFalconMotor2);

    } // end else if (USE_FALCON)

    // define drive

    m_drive = new DifferentialDrive(m_left, m_right);

    // code added to instantiate Spark MAX motor controllers (17 JAN 2020 PG)

    /**
     * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
     * 
     * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
     * first parameter
     * 
     * The motor type is passed as the second parameter. Motor type can either be:
     *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
     *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
     * 
     * The example below initializes six brushless motors with CAN IDs 4-6 and 7-9. Three motors 
     * are intended for the left side of the robot and three for the right side.
     */
    if (USE_NEO) {
      
      m_leftLeadNeoMotor = new CANSparkMax(LEFT_NEO_LEADER_ID, MotorType.kBrushless);
      m_leftFollowerNeoMotor1 = new CANSparkMax(LEFT_NEO_FOLLOWER1_ID, MotorType.kBrushless);
      m_leftFollowerNeoMotor2 = new CANSparkMax(LEFT_NEO_FOLLOWER2_ID, MotorType.kBrushless);

      m_rightLeadNeoMotor = new CANSparkMax(RIGHT_NEO_LEADER_ID, MotorType.kBrushless);
      m_rightFollowerNeoMotor1 = new CANSparkMax(RIGHT_NEO_FOLLOWER1_ID, MotorType.kBrushless);
      m_rightFollowerNeoMotor2 = new CANSparkMax(RIGHT_NEO_FOLLOWER2_ID, MotorType.kBrushless);

      // Best Practice: Apply controller settings using Java code

      m_leftLeadNeoMotor.restoreFactoryDefaults();
      m_leftFollowerNeoMotor1.restoreFactoryDefaults();
      m_leftFollowerNeoMotor2.restoreFactoryDefaults();

      m_rightLeadNeoMotor.restoreFactoryDefaults();
      m_rightFollowerNeoMotor1.restoreFactoryDefaults();
      m_rightFollowerNeoMotor2.restoreFactoryDefaults();

      /**
       * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
       * the follow() method on the SPARK MAX you want to configure as a follower, and by passing
       * as a parameter the SPARK MAX you want to configure as a leader.
       */
      m_leftFollowerNeoMotor1.follow(m_leftLeadNeoMotor);
      m_leftFollowerNeoMotor2.follow(m_leftLeadNeoMotor);
    
      m_rightFollowerNeoMotor1.follow(m_rightLeadNeoMotor);
      m_rightFollowerNeoMotor2.follow(m_rightLeadNeoMotor);
   
      m_robotDrive = new DifferentialDrive(m_leftLeadNeoMotor, m_rightLeadNeoMotor);

    } // end if
  
    else if (USE_FALCON) {
      // add falcon code here
    } // end else if

    // if needed, invert the entire right side
    
      m_right.setInverted(true);

} // end robotInit()

/**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
