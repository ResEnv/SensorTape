//import processing.core.*;

public class sensorNode {
  private int sensorType;
  private int numberOfSensors;
  private int [] orientation = {0,0,0,0}; 
  private int[][] inputData;
  private int sensorID; 
  private int arraySize = 10; 
  private int [] sum;
  int [] latestInput;
  int turn = 0; 
  int xpos = 0; 
  int ypos = 0;
  private int visualizationType = 1;
  boolean isSelected = false;
  
  public sensorNode(int sensorID, int numberOfSensors, int sensorType, int [] orientation) {
    inputData  = new int[numberOfSensors][arraySize];
    this.sensorID = sensorID; 
    this.sensorType = sensorType; 
    this.numberOfSensors = numberOfSensors ; 
    this.orientation = orientation;
    latestInput = new int [numberOfSensors]; 
    
    for (int i = 0; i < numberOfSensors; i++) latestInput[i] = 0;
    
  }
  
  public void addData(int input[]) { 
    latestInput = input;
      int tempArray []; 
      tempArray = new int[arraySize];
      sum = new int[arraySize];
      
      for (int g =0; g<numberOfSensors; g++) { 
        for(int i =1; i<arraySize; i++) { 
          tempArray[i] = inputData[g][i-1]; 
        }
        tempArray[0] = input[g]; 
        
        for (int h = 0; h<arraySize ; h++) { 
          inputData[g][h] = tempArray[h];               
        }
      }      
  }
  
  public int [] getLatest(){ 
    return latestInput; 
  }
  
  public int [] getAverage() {
    int [] latest = null; 
    return latest; 
  }
  
  
  public void printLatest() { 
    for (int i = 0 ; i<latestInput.length; i++){ 
      System.out.print("latest input:" + latestInput[i]);
    }
    System.out.println(" ");
  }
  
  public int [] getCutData() { 
    return orientation; 
    
  }
  
  public void setTurn(int newTurn) { 
    turn = newTurn;
  }
  public int getTurn() { 
    return turn;
  }
  
  public void printCutData() { 
    for (int i = 0 ; i<orientation.length; i++){ 
      System.out.print(orientation[i] +",");
    }
    System.out.println(" ");
  }
  
  public void setPosition(int x,int y) {
    xpos = x; 
    ypos  = y; 
  }
  
  public int getXPosition() { 
    return xpos;
  }
  
  public int getYPosition() { 
    return ypos;
  }
  
  public void setVisualizationType(int type) { 
    visualizationType = type;
  }
  
  public int getVisualizationType() { 
    return visualizationType;
  }
  
  public boolean isSelected() { 
    return isSelected;
  }
  
  public void setSelect(boolean select) {
    isSelected = select;
    
  }
}