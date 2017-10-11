package ca.mcgill.ecse211.localizationlab;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class LightLocalizer extends Thread {
  
  private static final int ROTATE_SPEED = 100; 
  private static double colorSensorDistance = 14.4; //How far the color sensor is from the center of rotation
  
  private double thetaY; //The angle that subtends the arc connecting the intersections of the light sensor's path with the y axis
  private double thetaY1;
  private double thetaY2;
  private double thetaX; //The angle that subtends the arc connecting the intersections of the light sensor's path with the y axis
  private double thetaX1;
  private double thetaX2;
  private double dTheta; //The angle needed to correct the odometer
  
  
  private Odometer odometer;
  private Navigation navigator;
  private UltrasonicPoller usPoller;
  private ColorSensorPoller csPoller;
  EV3LargeRegulatedMotor leftMotor;
  EV3LargeRegulatedMotor rightMotor;
  double leftRadius;                    //Holds the left wheel radius (cm)
  double rightRadius;                   //Holds the right wheel radius (cm)
  double width;  
  
  
  
  
  public LightLocalizer(Odometer odo, Navigation navigator, UltrasonicPoller usPoller, ColorSensorPoller csPoller, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double width){
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
          motor.stop();
          motor.setAcceleration(3000);}
    this.odometer = odo;
    this.navigator = navigator;
    this.usPoller = usPoller;
    this.csPoller = csPoller;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.leftRadius = leftRadius;
    this.rightRadius = rightRadius;
    this.width = width;
  }
  
  public void run(){
	  
	leftMotor.setAcceleration(300);
	rightMotor.setAcceleration(300);
    
    positionRobot();
    
    leftMotor.stop(true);
    rightMotor.stop(true);
    try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {      } 
    
    scan();
    
    leftMotor.stop(true);
    rightMotor.stop(true);
    try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {      } 

    calculate();
    try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {      } 
    
    navigator.travelTo(0,0);
    leftMotor.stop(true);
    rightMotor.stop(true);
    
    try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {      }
    //leftMotor.stop(true);
    //rightMotor.stop(true);
    navigator.turnTo(0 - odometer.getTheta()-10);
    
    //navigator.turnTo((-1)*this.odometer.getTheta());
    
  }
  
  
  private void positionRobot(){
    leftMotor.forward();
    rightMotor.forward();
    
    while(csPoller.getColor() > 50){
    //wait
    }
    //Sound.beep();
    leftMotor.stop(true);
    rightMotor.stop(true);
    try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {      } 
    leftMotor.backward();
    rightMotor.backward();
    try {
      Thread.sleep(5500);
    } catch (InterruptedException e) {      } 
    leftMotor.stop(true);
    rightMotor.stop(true);
    
    try {
      Thread.sleep(1500);
    } catch (InterruptedException e) {      } 
    
    leftMotor.rotate(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK , 90), true);
    rightMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK , 90), false);
    
    try {
      Thread.sleep(1500);
    } catch (InterruptedException e) {      } 
    
    rightMotor.forward();
    leftMotor.forward();
    
    while(csPoller.getColor() > 50){
      //wait
      }
    //Sound.beep();
    leftMotor.stop(true);
    rightMotor.stop(true);
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {      } 
    leftMotor.backward();
    rightMotor.backward();
    try {
      Thread.sleep(5500);
    } catch (InterruptedException e) {      } 
    
  }
  
  private void scan(){
    
    leftMotor.forward();
    rightMotor.backward();
    
    while(csPoller.getColor() > 50){
      leftMotor.forward();
      rightMotor.backward();
    }
    this.thetaY1 = odometer.getTheta();
    Sound.beep();
    while(csPoller.getColor() > 50){
      leftMotor.forward();
      rightMotor.backward();
    }
    this.thetaX1 = odometer.getTheta();
    Sound.beep();
    while(csPoller.getColor() > 50){
      leftMotor.forward();
      rightMotor.backward();
    }
    this.thetaY2 = odometer.getTheta();
    Sound.beep();
    while(csPoller.getColor() > 50){
      leftMotor.forward();
      rightMotor.backward();
    }
    leftMotor.stop(true);
    rightMotor.stop(true);
    this.thetaX2 = odometer.getTheta();
    Sound.twoBeeps();
    
    
  }
  
  private void calculate(){
    this.thetaY = Math.abs(this.thetaY2-this.thetaY1);
    this.thetaX = Math.abs(this.thetaX2-this.thetaX1);
    
    odometer.setX((-1)*(colorSensorDistance)*Math.cos((this.thetaY/360)*Math.PI));
    odometer.setY((-1)*(colorSensorDistance)*Math.cos((this.thetaX/360)*Math.PI));
    /*
    if(thetaY1 > 360){
      thetaY1 = thetaY1 - 360;
    }
    if(thetaY1 < 0){
      thetaY1 = thetaY1 + 360;
    }
 
    dTheta = (-90 + (thetaY1 - 180) - (thetaY/2))*(-1);
    
    leftMotor.rotate(+convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK , dTheta), true);
    rightMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK , dTheta), false);
    
    odometer.setTheta(0.0);
    */
 
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
  
  

}
