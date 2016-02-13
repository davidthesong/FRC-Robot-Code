package org.usfirst.frc.team295.robot;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
public class Robot extends SampleRobot {

    private RobotDrive myRobot; // robot drive system
    double Kp = 0.05;
    AHRS ahrs; 
    double distance;
    double time;
    Jaguar jaguarLeftBack;
	Jaguar jaguarLeftFront;
	Victor victorRightFront;
	Victor victorRightBack;
	Joystick joystick;
	float fSensorAngle;
    float fMotorAngle;
    float fzeroAngle;
    Double dSensorAngle;
    Double dMotorSpeed;
    Double dMotorAngle;
    double fMotorYaw;
    double joystickRaw;
    String driveDirection;
    double projectedAngle;
    public Robot() {
    	jaguarLeftBack = new Jaguar(2);
    	jaguarLeftFront = new Jaguar(3);
    	victorRightFront = new Victor(0);
    	victorRightBack = new Victor(1);
    	ahrs = new AHRS(SPI.Port.kMXP);          
        myRobot = new RobotDrive(
        	jaguarLeftBack,
        	jaguarLeftFront,
        	victorRightFront,
        	victorRightBack
  		);  // Drive train jaguars on PWM 1 and 2
        joystick = new Joystick(0);
        myRobot.setExpiration(0.1);
        /* 
         * Joystick Left Y-Axis Push up --> Negative Value Pull Down --> Positive Value
         * Must * -1 to get corrected value
         */
        
    }
    public void test(){
    	 double avg = 0;
         System.out.println("Test Started");
         fzeroAngle = ahrs.getYaw();
    	while(isTest() && isEnabled())
    	{
        	dSensorAngle = ahrs.getAngle(); // get current heading	            
            fMotorYaw = ahrs.getYaw() - fzeroAngle;
            avg = avg - avg/8 + ahrs.getYaw()/8;
            SmartDashboard.putNumber("Zero", fzeroAngle);
            SmartDashboard.putNumber("Angle", dSensorAngle);
    		SmartDashboard.putNumber("Yaw", ahrs.getYaw());
    		SmartDashboard.putNumber("Manual Yaw", fMotorYaw);
    		SmartDashboard.putNumber("Average", avg);
    		if(joystick.getRawButton(1)){
//    			ahrs.reset();
    			ahrs.reset();
    			fzeroAngle = ahrs.getYaw();
    		}
//    		myRobot.drive(deadband(-joystick.getRawAxis(1)), -ahrs.getYaw()*Kp);
    		myRobot.tankDrive(-1 * joystick.getRawAxis(1), -1 * joystick.getRawAxis(5));
//    		if(joystick.getRawAxis(1)>0){
//    			driveDirection = "Forwards";
//    			System.out.println("Direction: " +  driveDirection);
//    		}else if(joystick.getRawAxis(1)<0){
//    			driveDirection = "Backwards";
//    			System.out.println("Direction: " +  driveDirection);
//    		}
    		
    	}
    }
    public void operatorControl(){
    	
        double avg = 0;
        System.out.println("Teleop Started");
        fzeroAngle = ahrs.getYaw();
        projectedAngle = ahrs.getYaw() + 90;
        double diff;
        double turn;
        double pointAngle = 0;
        double initTime = Timer.getFPGATimestamp();
        while(isOperatorControl() && isEnabled()){
        	dSensorAngle = ahrs.getAngle(); // get current heading	            
            fMotorYaw = ahrs.getYaw() - fzeroAngle;
            avg = avg - avg/8 + ahrs.getYaw()/8;
            joystickRaw = -joystick.getRawAxis(1);
            SmartDashboard.putNumber("Zero", fzeroAngle);
            SmartDashboard.putNumber("Angle", dSensorAngle);
    		SmartDashboard.putNumber("Yaw", ahrs.getYaw());
    		SmartDashboard.putNumber("Manual Yaw", fMotorYaw);
    		SmartDashboard.putNumber("Average", avg);
    		SmartDashboard.putNumber("Projected Yaw",projectedAngle);
    		if(joystick.getRawButton(1)){
    			pointAngle = ahrs.getYaw();
    			projectedAngle = pointAngle + 90;
    			initTime = Timer.getFPGATimestamp();
    		}
    		if(Timer.getFPGATimestamp()>initTime){
    			initTime += 0.020;
    			if(pointAngle<projectedAngle){
    				pointAngle +=1;
    			}
    		}
    		SmartDashboard.putNumber("seconds", Timer.getFPGATimestamp());
//    		if(joystickRaw < 0){
//    			myRobot.drive(deadband(joystickRaw), avg*Kp);
//    		}
//    		else{
//    			myRobot.drive(deadband(joystickRaw),-avg*Kp);
//    		}
//    		
    		diff = pointAngle - avg;
    		
    		if(joystickRaw > 0 && projectedAngle>0 && Math.abs(diff)>5){
    			turn = (diff)* Kp;
    			myRobot.drive(deadband(joystickRaw), 1);
    			SmartDashboard.putNumber("diff",diff);
    		}
//    		myRobot.drive(deadband(joystickRaw), deadband(-1*joystick.getRawAxis(5)));
//    		myRobot.tankDrive(-1 * joystick.getRawAxis(1), -1 * joystick.getRawAxis(5));
        }
        if(isDisabled()){
        	ahrs.reset();
        }
    	
    }
    private double deadband(double d) {
		// TODO Auto-generated method stub
    	if(Math.abs(d) < .15){
    		return 0;
    	}
    	else{
    		return d;
    	}
		
	}
	public void autonomous() {
      
//      ahrs.reset();
      fzeroAngle = ahrs.getYaw();
      time = Timer.getFPGATimestamp();
      ahrs.reset();
        while (isAutonomous()) {
//        	dSensorAngle = ahrs.getAngle(); // get current heading	            
//            dMotorSpeed = -1.0;
//            dMotorAngle = dSensorAngle * Kp;
//            fMotorYaw = ahrs.getYaw()-fzeroAngle;            
//            SmartDashboard.putNumber("Computed Angle", dMotorAngle);
            SmartDashboard.putNumber("Compass Fused Heading", ahrs.getFusedHeading());
    		SmartDashboard.putNumber("Compass Heading", ahrs.getCompassHeading()); //Returns 0 because of magnetic interference? not calibrated
//    		SmartDashboard.putNumber("Angle", dSensorAngle);
    		SmartDashboard.putNumber("Yaw", ahrs.getYaw());
    		SmartDashboard.putNumber("Manual Yaw", fMotorYaw);
    		
    		myRobot.tankDrive(-1 * joystick.getRawAxis(1), -1 * joystick.getRawAxis(5));
//    		myRobot.drive(joystick.getRawAxis(1), -fMotorYaw*Kp);
    		if(joystick.getRawButton(1)){
//    			ahrs.reset();
    			ahrs.reset();
    			
    		}
    		Timer.delay(0.004);
    		
        }
        myRobot.drive(0.0, 0.0);
        
    }
}
