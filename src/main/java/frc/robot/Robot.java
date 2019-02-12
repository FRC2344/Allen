/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.F310;
import jaci.pathfinder.followers.EncoderFollower;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /*- Drive Train -*/
  WPI_TalonSRX leftMaster, leftPrimarySlave, rightMaster, rightPrimarySlave, leftSecondarySlave, rightSecondarySlave, elevatorMain;
  DifferentialDrive drive;

  /*- Autonomous -*/
  //PigeonIMU pigey;
  //PigeonIMU.FusionStatus fusionStatus;
  Timer autoTimer;
  Compressor comp;
  Solenoid shift;
  
  enum AutonomousSteps{
    DRIVE_AWAY_FROM_HAB,
    TURN,
    DRIVE_TO_CARGO_SHIP,
    DUMP_BALL,
    RETURN_TO_HAB_PLATFORM
  }

  enum views{
    horizontal,
    vertical
  }

  AutonomousSteps autosteps;
 /*- Pathfinder -*/
 EncoderFollower m_left_follower;
 EncoderFollower m_right_follower;
 Notifier m_follower_notifier;

  /*- Joysticks -*/
  Joystick driver, operator;
  F310 joy;

  enum Angle{
    LEFT,
    RIGHT
  }
 
  @Override
  public void robotInit() {
    //Drive Train Setup
    //- Historically, we use TalonSRX as our encoders to let us communicate to 
    //- our motors. They are usually setup in a master and slave environment. 
    leftMaster = new WPI_TalonSRX(1);
    leftPrimarySlave = new WPI_TalonSRX(2);
    rightMaster = new WPI_TalonSRX(4);
    rightPrimarySlave = new WPI_TalonSRX(5);
    leftSecondarySlave = new WPI_TalonSRX(3);
    rightSecondarySlave = new WPI_TalonSRX(6);
    elevatorMain = new WPI_TalonSRX(8);
    //Talon SRX Configurations
    //- We need to actually begin to define the slaves, and communicate to them
    //- to follow the masters, as well as invert too.

    //Left Master
      leftMaster.setNeutralMode(NeutralMode.Coast);
      leftMaster.setInverted(false);
    //Right Master
       rightMaster.setNeutralMode(NeutralMode.Coast);
       rightMaster.setInverted(false);
       rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
       rightMaster.setSelectedSensorPosition(0);
    //Left Slave
        leftPrimarySlave.setNeutralMode(NeutralMode.Coast);
        leftPrimarySlave.follow(this.leftMaster);
        leftPrimarySlave.setInverted(InvertType.FollowMaster);
        leftPrimarySlave.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        leftPrimarySlave.setSelectedSensorPosition(0);
    //Right Slave
        rightPrimarySlave.setNeutralMode(NeutralMode.Coast);
        rightPrimarySlave.follow(this.rightMaster);
        rightPrimarySlave.setInverted(InvertType.FollowMaster);
        rightSecondarySlave.setNeutralMode(NeutralMode.Coast);
        rightSecondarySlave.follow(this.rightMaster);
        rightSecondarySlave.setInverted(InvertType.FollowMaster);
        leftSecondarySlave.setNeutralMode(NeutralMode.Coast);
        leftSecondarySlave.follow(this.rightMaster);
        leftSecondarySlave.setInverted(InvertType.FollowMaster);
    

    //Differential Drive
    //- The Differential Drive Class, allows us to actually begin to communicate and
    //- drive the robot.
    drive = new DifferentialDrive(leftMaster, rightMaster);
    driver = new Joystick(0);

    //Autonomous
    //- Timer, this will allow us to delay some functions.
    autoTimer = new Timer();
    joy = new F310();
    //pigey = new PigeonIMU(rightPrimarySlave);
    //fusionStatus = new PigeonIMU.FusionStatus();
    //  pigey.setFusedHeading(0.0, 30);
    operator = new Joystick(1);
    comp = new Compressor();
    shift = new Solenoid(3);
  }

  @Override
  public void robotPeriodic() {
    boolean tru_e = false;
    boolean shift_mode = false;
   // pigey.getFusedHeading(fusionStatus);
    drive.setSafetyEnabled(false);
   // double currentAngle = fusionStatus.heading;
   // SmartDashboard.putNumber("Pigeon Angle", currentAngle);
    SmartDashboard.putNumber("Right Joystick", driver.getRawAxis(2));
    SmartDashboard.putNumber("Left Positon", leftPrimarySlave.getSelectedSensorPosition(0) );
    SmartDashboard.putNumber("Right Positon", rightMaster.getSelectedSensorPosition(0) );
    if(driver.getRawButtonPressed(8)){
      tru_e = true;
    }

    if(tru_e){
      rightMaster.setSelectedSensorPosition(0);
      leftPrimarySlave.setSelectedSensorPosition(0);
     // pigey.setFusedHeading(0);
      tru_e = false;
    }

    if(driver.getRawButtonPressed(7)){
      shift.set(true);
    }

    if(driver.getRawButtonReleased(7)){
      shift.set(false);
    }

    




  }

  @Override
  public void autonomousInit() {
    rightMaster.setSelectedSensorPosition(0);
    leftPrimarySlave.setSelectedSensorPosition(0);
    autosteps = AutonomousSteps.DRIVE_AWAY_FROM_HAB;
   /* Trajectory left_trajectory = PathfinderFRC.getTrajectory("Auto.left");
    Trajectory right_trajectory = PathfinderFRC.getTrajectory("Auto.right");

    m_left_follower = new EncoderFollower(left_trajectory);
    m_right_follower = new EncoderFollower(right_trajectory);

    m_left_follower.configureEncoder(leftPrimarySlave.getSelectedSensorPosition(0), 1037, 6.0);
    m_right_follower.configureEncoder(rightMaster.getSelectedSensorPosition(0), 1037, 6.0);
    m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / 10, 0);
    m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 /10, 0);

    m_follower_notifier = new Notifier(this::followPath);

    m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
    */
  }

 /* private void followPath(){
    if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
      m_follower_notifier.stop();
    } else {
      double left_speed = m_left_follower.calculate(leftPrimarySlave.getSelectedSensorPosition(0));
      double right_speed = m_right_follower.calculate(rightPrimarySlave.getSelectedSensorPosition(0));
      double heading = fusionStatus.heading;
      double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn =  0.8 * (-1.0/80.0) * heading_difference;
      drive.tankDrive((left_speed + turn), (right_speed - turn), false);
    }
  }*/

  @Override
  public void autonomousPeriodic() {
   switch(autosteps){
      case DRIVE_AWAY_FROM_HAB:
      if(getSensorPosition() < Sensor2Feet(9)){
        drive.arcadeDrive(-.5, 0, false);
      }
      else if(getSensorPosition() >= Sensor2Feet(9)){
         drive.arcadeDrive(0, 0, false);
         autosteps = AutonomousSteps.TURN;
        }
      break;
      case TURN:
      //if(fusionStatus.heading < angle(Angle.RIGHT, 90) ){
       //   drive.arcadeDrive(0, -.25, false);
     // } 
    //  else if(fusionStatus.heading >= angle(Angle.RIGHT, 90) ){ 
     //   drive.arcadeDrive(0, 0, false);
     //     pigey.setFusedHeading(0.0, 30);
      //    rightMaster.setSelectedSensorPosition(0);
       //   leftPrimarySlave.setSelectedSensorPosition(0);
       //   autosteps = AutonomousSteps.DRIVE_TO_CARGO_SHIP;
    //  }
      break;
      case DRIVE_TO_CARGO_SHIP:
      if(getSensorPosition() < Sensor2Feet(9)){
        double k_angle = 0.09;
  
          //drive.arcadeDrive(-.75, fusionStatus.heading * k_angle, false);
        }
       else if(getSensorPosition() >= Sensor2Feet(9)){
        drive.stopMotor();
        autosteps = AutonomousSteps.DUMP_BALL;
      }
      break;
      case DUMP_BALL:
        drive.stopMotor();
      break;
      case RETURN_TO_HAB_PLATFORM:
      break;
  
    }     


  }

  @Override
  public void teleopInit() {
    autoTimer.stop();
  }

  @Override
  public void teleopPeriodic() {
    //drive.arcadeDrive(joy.getLeftJoystick(driver), driver.getRawAxis(2));
    drive.arcadeDrive(-joy.getLeftJoystick(driver), driver.getRawAxis(2));
    elevatorMain.set(-joy.getLeftJoystick(operator));
    
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  public double getSensorPosition(){
    return rightMaster.getSelectedSensorPosition(0);
  }
  
  public double Sensor2Feet(int feet){
    int one_rev = 789, overflow = 1203;
    double answer;

    answer = one_rev * feet;
    answer = answer - overflow;
    return answer;
  }


  

  public double angle(Angle x, double a){

    if(x == Angle.LEFT){
      return -a;
    }
    else if(x == Angle.RIGHT){
      return a;
    }

    return 0;


  }
  

}
