package ca.mcgill.ecse211.localizationlab;

import lejos.hardware.Button;

public class UltrasonicLocalizer extends Thread {
  
  private int mode;
  
  public void run(){
    
    if(mode == 1){    //Falling Edge Mode
    
      this.fallingEdge();  
    
    }
    
    if(mode == 2){    //Rising Edge Mode
      
      this.risingEdge();
      
    }
    
    
  }

  public void fallingEdge(){

  }
  
  
  public void risingEdge(){
    
  }
  
  public void setMode(int mode){
    
    this.mode = mode;
    
  }
  

}
