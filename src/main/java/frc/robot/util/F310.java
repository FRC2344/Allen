/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
public class F310 {

  /*
  * AXIS
  */
    public double getLeftJoystick(Joystick _joy){
        return -_joy.getRawAxis(1);
      }
  
      public double getRightJoystick(Joystick _joy){
        return _joy.getRawAxis(3);
      }
  /*
  * BUTTONS
  */
      public boolean getX(Joystick _joy){
        return _joy.getRawButton(1);
      }
  
      public boolean XPressed(Joystick _joy){
        return _joy.getRawButtonPressed(1);
      }
  
      public boolean getA(Joystick _joy){
        return _joy.getRawButton(2);
      }
  
      public boolean APressed(Joystick _joy){
        return _joy.getRawButtonPressed(2);
      }
  
      public boolean getB(Joystick _joy){
        return _joy.getRawButton(3);
      }
  
      public boolean BPressed(Joystick _joy){
        return _joy.getRawButtonPressed(3);
      }
  
      public boolean getY(Joystick _joy){
        return _joy.getRawButton(4);
      }
  
      public boolean YPressed(Joystick _joy){
        return _joy.getRawButtonPressed(4);
      }
  /*
  * TRIGGERS
  */
      public boolean getLB(Joystick _joy){
        return _joy.getRawButton(1);
      }
  
      public boolean LBPressed(Joystick _joy){
        return _joy.getRawButtonPressed(1);
      }
      public boolean getRB(Joystick _joy){
        return _joy.getRawButton(1);
      }
  
      public boolean RBPressed(Joystick _joy){
        return _joy.getRawButtonPressed(1);
      }
      public boolean getLT(Joystick _joy){
        return _joy.getRawButton(1);
      }
  
      public boolean LTPressed(Joystick _joy){
        return _joy.getRawButtonPressed(1);
      }
      public boolean getRT(Joystick _joy){
        return _joy.getRawButton(1);
      }
  
      public boolean RTPressed(Joystick _joy){
        return _joy.getRawButtonPressed(1);
      }
  
}
