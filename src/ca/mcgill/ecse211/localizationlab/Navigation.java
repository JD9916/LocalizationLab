package ca.mcgill.ecse211.localizationlab;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.Sound;

public class Navigation extends Thread{
	
  EV3LargeRegulatedMotor leftMotor;
  EV3LargeRegulatedMotor rightMotor;
	
  double leftRadius;                    //Holds the left wheel radius (cm)
  double rightRadius;                   //Holds the right wheel radius (cm)
  double width;                         //Holds the length of the wheel base (cm)
	
  private static final int FORWARD_SPEED = 100;     //Default drive speed (deg/sec)
  private static final int ROTATE_SPEED = 50;       //Default rotate speed (deg/sec)
  private static final int MINIMUM_DISTANCE = 25;   //Minimum distance to activate avoidance (cm)
  private boolean isNavigating = false;             //Checks if travelTo() or turnTo() is being called.
  private double toTheta;               //Used to correct the robot's orientation (in degrees)
  private Odometer odometer;
  private boolean navigating = false;   //Initially, the robot is not navigating
  public double x, y;                   //The desired x and y coordinates of the next checkpoint.
  private double points[][] = {{0.0,0.0}};;
	
   public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double width) {
     //Here, objects instantiated in NavigationLab are passed to the Navigation Class.
     this.odometer = odo;
     this.leftMotor = leftMotor;
     this.rightMotor = rightMotor;
     this.leftRadius = leftRadius;
     this.rightRadius = rightRadius;
     this.width = width;
	 }
	
  public void run(){
    for (double[] p : points){
	  try {
	    Thread.sleep(1000);        //Wait 1s before navigating (after button is pushed)
	  } catch (InterruptedException e) { 		  
	  }                        
	  x = p[0];                    //Load the next checkpoint from
	  y = p[1];                    //points into x and y
	  this.isNavigating = true;    //The robot will now be navigating.
	  this.travelTo(p[0], p[1]);   //Travels to the next point
	  this.isNavigating = false;   //The robot has finished navigating
      }
    leftMotor.stop(true);              //Stops motors after end is reached.
    rightMotor.stop(true);
	}

  public void travelTo(double x, double y) {
    
    //toTheta will store the angle the robot must orient itself to point in the direction
    //of the next checkpoint
    toTheta = this.getAngle(x, y) - odometer.getTheta();
	  if(toTheta > 180){           //The correction angle must always be the acute angle.
	    toTheta -= 360.0;          //If it is not, then it is turned into one.
	  }else if(toTheta < -180){
	    toTheta += 360.0;
	  }
      if((this.odometer.getTheta() <= (this.getAngle(x, y) -0.5)) 
        || (this.odometer.getTheta() >= (this.getAngle(x, y) + 0.5)) ){
      //checks to see if the robot is not facing the right way
        turnTo(toTheta);            //Corrects the bot's orientation
	  }
	  while(this.stop(x, y) == 0){  //While the robot has not reached its next checkpoint.
	    if(UltrasonicPoller.getDistance() < MINIMUM_DISTANCE){
		  double beforeTheta;     //The robots orientation before avoidance
		  double afterTheta;      //The robots orientation after facing away from the wall
		  double deltaTheta;      //The diff between beforeTheta and afterTheta
		  double correction;      //How far the robot must drive to avoid the obstacle
		  double distanceFromWall;//How far the robot is from the wall
		  distanceFromWall = UltrasonicPoller.getDistance();
		  
		  //This avoidance algorithm works by
		  //1. Turning until it sees no object
		  //2. Drive forward until past the object
		  //3. Turn back the necessary angle to come behind the object
		  //4. Drive back behind the object
		  
		  
		  /*while (1==1){
		    beforeTheta = odometer.getTheta();     //Get the robot's initial orientation
		    while(UltrasonicPoller.getDistance() < 40){
		      //Here, the robot will turn until it sees no wall
			  leftMotor.rotate(convertAngle(leftRadius, width, 5), true);
			  rightMotor.rotate(-convertAngle(rightRadius, width, 5), false);
			}
		    //Once the robot sees no wall, it turns an extra 10deg to make sure it will
		    //not come close to the wall when driving straight
			leftMotor.rotate(convertAngle(leftRadius, width, 10), true);
			rightMotor.rotate(-convertAngle(rightRadius, width, 10), false);
			
			afterTheta = odometer.getTheta();     //Get the robot's new orientation
            deltaTheta = Math.abs(beforeTheta-afterTheta);  //Compute deltaTheta
            //Compute how far the robot must travel to drive past the wall
            correction = distanceFromWall/Math.cos(deltaTheta*((Math.PI)/(180)));
            //Drive past the wall
            leftMotor.rotate(convertDistance(leftRadius, correction), true);
            rightMotor.rotate(convertDistance(rightRadius, correction), false);
            //The two lines below tell the robot to drive an extra distance forward to
            //composate for the width of the walls
            leftMotor.rotate(convertDistance(leftRadius, 8), true);
            rightMotor.rotate(convertDistance(rightRadius, 8), false);
            //The robot turns the necessary angle to point itself behind the wall
            leftMotor.rotate(-convertAngle(leftRadius, width, 180-2*deltaTheta), true);
            rightMotor.rotate(convertAngle(rightRadius, width, 180-2*deltaTheta), false);
            //The robot drives straight to behind the wall, and it is now past it.
            leftMotor.rotate(convertDistance(leftRadius, correction), true);
            rightMotor.rotate(convertDistance(rightRadius, correction), false);
            //Now the robot must orient itself towards the coordinates again.
            toTheta = this.getAngle(x, y) - odometer.getTheta();
            if(toTheta > 180){
			  toTheta -= 360.0;
			}else if(toTheta < -180){
			  toTheta += 360.0;
			}
            if((this.odometer.getTheta() <= (this.getAngle(x, y) -1)) || (this.odometer.getTheta() >= (this.getAngle(x, y) + 1)) ){
              turnTo(toTheta);
            }
            break;
		  }*/	
	    }
	    //If facing the right angle, and no object is detected, move forward.
		leftMotor.setSpeed(FORWARD_SPEED);
	    rightMotor.setSpeed(FORWARD_SPEED);
	    leftMotor.forward();
	    rightMotor.forward();
	    leftMotor.rotate(convertDistance(LocalizationLab.WHEEL_RADIUS, Math.sqrt(Math.pow(odometer.getX() - x,2) + Math.pow(odometer.getY() - y,2))), true);
	    rightMotor.rotate(convertDistance(LocalizationLab.WHEEL_RADIUS, Math.sqrt(Math.pow(odometer.getX() - x,2) + Math.pow(odometer.getY() - y,2))), false);
	   
      }	
	  Sound.beep();      //Make a beeping sound
	}
	
    public void turnTo(double theta) {
	  this.navigating = true;              //Sets the robot to navigation mode
      leftMotor.setSpeed(ROTATE_SPEED);    //Sets the motors to rotation speed  
      rightMotor.setSpeed(ROTATE_SPEED);
      
      //Turns both motors by the correction angle of theta (in degrees).
      leftMotor.rotate(convertAngle(leftRadius, width, theta), true);
      rightMotor.rotate(-convertAngle(rightRadius, width, theta), false);
      this.navigating = false;              //Turns navigation mode off
    }
	
	public double getAngle(double x, double y){
      double theta;
      //Calculates the angle the robot must orient itself in to face the checkpoint.
      //This is done using the robot's current position, the checkpoint position,
      //and simple trigonometry.
	  double deltaX = x - this.odometer.getX();
	  double deltaY = y - this.odometer.getY();
	  //note: The angle must be converted from radians to degrees.
	  theta = 90-(Math.atan(deltaY/deltaX)*(180/Math.PI)); 
	  if(deltaX < 0) {
	    theta += 180;  //This correction is included to fix negative angles.
	  }
    return theta;
	}
	
	private int stop(double x, double y) {
	  //This method is used to check if the robot has reached it's next
	  //coordinates yet. Naturally, there will need to be some room of error.
      int stop = 1;  //Initially set stop to true.
      if(this.odometer.getX() < x - 2 || this.odometer.getX() > x + 2){
      stop = 0;     //If the X coordinate is not correct, set stop to false.
      }
      if(this.odometer.getY() < y - 2 || this.odometer.getY() > y + 2){
        stop = 0;   //If the Y coordinate is not correct, set stop to false.
      }
      return stop;
	  }

	private static int convertAngle(double radius, double width, double angle) {
	  //Given wheel radius (cm), wheel base (cm), and desired turn angle (degrees)
	  //converts to the angle the wheels need to drive.
	  return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	private static int convertDistance(double radius, double distance) {
	  //Given the distance (cm) to travel and the wheel radius (cm), 
	  //converts to amount of wheel rotation needed.
	  return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private boolean isNavigating() {
	  //Returns true if travelTo() or turnTo() have been called and not yet returned
	  return this.isNavigating;
	}
}
