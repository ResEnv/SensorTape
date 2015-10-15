import java.text.DecimalFormat;
import toxi.geom.*;

public class visualize {
  private PApplet applet;
  
  int u_ctrl_pts;;
  int v_ctrl_pts; 
  int u_knotsN;  
  int v_knotsN;  

  float u_knotsSpacing;
  float v_knotsSpacing; 

  float [] u_knots;
  float [] v_knots;
   
  PVector [][] ctrl_pts;
    
  float u_spacing;
  float v_spacing;
  
  float xglobal = 0 ; 
  float yglobal = 0 ;
  
  //This passes the applet here, but only needed in Eclipse Java
  public visualize(PApplet _applet) {
    applet = _applet;
  }
  
  public void initiate(int nOfNodes) { 
    
       u_ctrl_pts = nOfNodes;
       v_ctrl_pts = 3 ;
    
       u_knotsN = u_ctrl_pts+3;  //Number of control points + curve degree + 1; 
       v_knotsN = v_ctrl_pts+3; 
      
        u_knotsSpacing = 1 / ((float)u_knotsN-1); 
        v_knotsSpacing = 1 / ((float)v_knotsN-1); 
        
        u_knots = new float[u_knotsN]; 
        v_knots = new float[v_knotsN]; 
    
        for (int i=0; i<u_knotsN; i++) { 
            u_knots[i] = i*u_knotsSpacing; 
            
         } 
         for (int i=0; i<v_knotsN; i++){ 
            v_knots[i] = i*v_knotsSpacing; 
          } 
    
      ctrl_pts = new PVector[u_ctrl_pts][v_ctrl_pts];
      
      // set up control points in a regular grid on the xz plane with a random height:
      u_spacing = (applet.width / u_ctrl_pts);
       v_spacing = (applet.width / v_ctrl_pts);
     
      for (int i = 0; i < u_ctrl_pts; i++) {
        for (int j = 0; j < v_ctrl_pts; j++) {
           ctrl_pts[i][j] = new PVector( u_spacing * i, 600, -v_spacing * j);
        }
      }  
  }//end initiate
  
  public void drawSensors(ArrayList <sensorNode>listSensors, int type) { 
      for (int i = 0; i<listSensors.size(); i++) { 
        if(i>0) { 
            drawOneSensor(i, listSensors.get(i).getLatest(), type,listSensors.get(i).getTurn(), 
            listSensors.get(i).getXPosition(), listSensors.get(i).getYPosition(), 
            listSensors.get(i-1).getXPosition(), listSensors.get(i-1).getYPosition(), 
            listSensors.get(i-1).getLatest());
        }
        else  { 
          drawOneSensor(i, listSensors.get(i).getLatest(), type,listSensors.get(i).getTurn(), 
               listSensors.get(i).getXPosition(), listSensors.get(i).getYPosition(), 
             listSensors.get(i).getXPosition(), listSensors.get(i).getYPosition(),
             listSensors.get(i).getLatest());
        }
      }  
      xglobal = 100; 
      yglobal = 0 ;
  }
  
  public void drawOneSensor(int position, int[] data, int type, int orientation, int xpos,int ypos, int xprev, int yprev, int[] prevData ) { 
    
      //---------Visualize light sensor---------
      if (type ==1 && data.length>7) {
        fill(data[1],0,0); 
        rect(xpos,ypos, 60,60); 
        textSize(15);
        text("Light Sensor", 100, 200);
      }   
      //---------Visualize proximity---------
      if (type ==2 && data.length>7) { 
        fill(0,0,data[3]); 
        text(data[3] +" mm", xpos, ypos);
        rect(xpos,ypos, 60,60); 
        text("Proximity Sensor", 100, 200);
      }
      
      //---------Visualize IMU data, in quaternion format---------
      if (type==3 && data.length>7) { 
        text(data[4], xpos, ypos);
        text(data[5], xpos, ypos-20);
        text(data[6], xpos, ypos-40);
        text(data[7], xpos, ypos-60);
        fill(128);
        rect(xpos,ypos, 60,60);   
        text("IMU Sensor", 100, 200);
      }
      //---------Visualize temperature---------
      if (type==4 && data.length>7) { 
        //does not convert it right, needs better calibration
        text(convertToTemperature(data[2]) + " C",xpos, ypos);
        fill(128);
        rect(xpos,ypos, 60,60);   
        text("Temperature Sensor", 100, 200);
      } 
      //---------3D Visualization---------
      if (type==5 && data.length>7) {
        applet.pushMatrix(); 

        if (data.length>5) {
          //Get the raw orientation data in the quaternion format. 
          float[] q = new float[4];
          Quaternion quat = new Quaternion(1, 0, 0, 0);
          q[0] = (data[4]) / 16384.0f;
          q[1] = (data[5]) / 16384.0f;
          q[2] = (data[6]) / 16384.0f;
          q[3] = (data[7]) / 16384.0f;
          for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i];
          quat.set(q[0], q[1], q[2], q[3]);
    
          //calculate roll, pitch, yaw from the quaternion. 
          //I am using roll, pitch, yaw (Tait-Bryan angles) so it is orientation indifferent
          float roll,pitch,yaw;
          roll = applet.atan2((float) (2.0*(quat.y*quat.z + quat.w*quat.x)), quat.w*quat.w - quat.x*quat.x - quat.y*quat.y + quat.z*quat.z);
          pitch = (float) (-1.0 * applet.asin((float) (-2.0*(quat.x*quat.z - quat.w*quat.y))));
          yaw = applet.atan2((float) (2.0*(quat.x*quat.y + quat.w*quat.z)), quat.w*quat.w + quat.x*quat.x - 
              quat.y*quat.y - quat.z*quat.z); 
            
          //Calculate the new X,Y,Z coordinates from the pitch, roll, yaw    
          float newPosX = xglobal+120*applet.cos(roll); 
          float newPosY = (yglobal)+120*applet.sin(roll); 
          float twistY = 1+150*applet.sin(pitch); 
          float twistZ = 1+150*applet.cos(pitch);           
  
          //For debugging, print data from one node
          /*
          if(data[0]==12) {
              applet.text("roll: " + degrees(roll),600,200); 
              applet.text("pitch: " + degrees(pitch), 600, 250);
              applet.text("yaw: " + degrees(yaw), 600, 300);
              applet.println("newPosX: ", newPosX,600,350);
              applet.println("newPosY:", newPosY,600,400);  
              applet.text("twistY: " + twistY, 600,450); 
              applet.text("twistZ: " + twistZ, 600, 500);
          }//end for
          */
          
          //Draw the skeleton line  
          applet.strokeWeight(5);  
          applet.line(xglobal,yglobal+600,newPosX,newPosY+600);
          applet.stroke(128);
          applet.strokeWeight(20);  // Beastly
          applet.stroke(255,0,0);
          applet.point(newPosX, newPosY+600);
          applet.fill(255, 20, 20);
          xglobal = newPosX; 
          yglobal = newPosY; 
            
          applet.translate(-2900,1600,-2200); //z-lower smaller, x - lower right, y - higher number, lower
          
          int u_deg = u_knots.length - u_ctrl_pts - 1;
          int v_deg = v_knots.length - v_ctrl_pts - 1;
           
          // draw the surface
          for (float u = u_knots[u_deg]; u <= u_knots[u_knots.length-u_deg-1] - 0.01; u += 0.01) {
            applet.beginShape(applet.QUAD_STRIP);
            for (float v = v_knots[v_deg]; v <= v_knots[v_knots.length-v_deg-1]; v += 0.01) {
              PVector pt_uv = new PVector();
              PVector pt_u1v = new PVector(); // u plus 0.01
              for (int i = 0; i < u_ctrl_pts; i++) {
                for (int j = 0; j < v_ctrl_pts; j++) {
                  float basisv = basisn(v,j,v_deg,v_knots);
                  float basisu = basisn(u,i,u_deg,u_knots);
                  float basisu1 = basisn((float) (u+0.01),i,u_deg,u_knots);
                  PVector pk = PVector.mult( ctrl_pts[i][j], basisu * basisv);
                  PVector pk1 = PVector.mult( ctrl_pts[i][j], basisu1 * basisv);
                  pt_uv.add( pk );
                  pt_u1v.add( pk1 );
                }//end for
              }//end for
              applet.noStroke(); 
              applet.fill( 255 );
              applet.vertex( pt_uv.x, pt_uv.y, pt_uv.z );
              applet.vertex( pt_u1v.x, pt_u1v.y, pt_u1v.z );
            }//end for
            applet.endShape();
          }//end for
  
          //place the control points (x,y,z) and add a twist offset. 
          ctrl_pts[position][2] = new PVector( newPosX*4 , newPosY*4+4*twistY, 2*-v_spacing + twistZ*4);
          ctrl_pts[position][1] = new PVector( newPosX*4 , newPosY*4, 1* -v_spacing * 1/5);
          ctrl_pts[position][0] = new PVector( newPosX*4 , newPosY*4-4*twistY, 0* -v_spacing + twistZ*4);
          
          //Visualize the control points (not required)         
          for (int i = 0; i < u_ctrl_pts; i++) {
              for (int j = 0; j < v_ctrl_pts; j++) {
                applet.stroke(255,0,0);
                applet.strokeWeight(10);  // Beastly
                applet.point(ctrl_pts[i][j].x, ctrl_pts[i][j].y,ctrl_pts[i][j].z);
                applet.strokeWeight(1);    
              }//end for
           }//end for
        }else  
        { 
         // Do nothing
        }
        applet.popMatrix(); 
        applet.stroke(126);
        applet.line(30, 20, 85, 75);
        }
  }//end drawOneSensor
  
  float basisn(float u, int k, int d, float [] knots)
  {
    if (d == 0) {
      return basis0(u,k,knots);
    }
    else {
      float b1 = basisn(u,k,d-1,knots) * (u - knots[k]) / (knots[k+d] - knots[k]);
      float b2 = basisn(u,k+1,d-1,knots) * (knots[k+d+1] - u) / (knots[k+d+1] - knots[k+1]);
      return b1 + b2;
    }
  }//end basisn
   
  float basis0(float u, int k, float [] knots)
  {
    if (u >= knots[k] && u < knots[k+1]) {
      return 1;
    }
    else {
      return 0;
    } 
  }//end basis0
  
  
  
   String convertToTemperature(int RawADC) {
     double Temp;
     Temp = applet.log((float) (10000.0*((1024.0/RawADC-1)))); 
//             =log(10000.0/(1024.0/RawADC-1)) // for pull-up configuration
     Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
     Temp = Temp - 273.15;            // Convert Kelvin to Celcius
    // Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
    DecimalFormat df = new DecimalFormat("#.0");
    String formatedString = df.format(Temp);          
    return formatedString;
   }//end covertToTemperature
}