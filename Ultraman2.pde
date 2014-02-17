/* --------------------------------------------------------------------------
 * SimpleOpenNI User Test
 * --------------------------------------------------------------------------
 * Processing Wrapper for the OpenNI/Kinect library
 * http://code.google.com/p/simple-openni
 * --------------------------------------------------------------------------
 * prog:  Max Rheiner / Interaction Design / zhdk / http://iad.zhdk.ch/
 * date:  02/16/2011 (m/d/y)
 * ----------------------------------------------------------------------------
 */

import SimpleOpenNI.*;
import java.util.Map;
import java.util.HashMap;

SimpleOpenNI  context;
boolean       autoCalib=true;

PImage cam;
PImage maskImage;
PImage skinImage;

int startY = -1;

//int[] userMap; /* store User pixels */

Map<Integer, PVector> neckPos = new HashMap(); /* store neck position */

void setup()
{
  context = new SimpleOpenNI(this);
   
  // enable depthMap generation 
  if(context.enableDepth() == false)
  {
     println("Can't open the depthMap, maybe the camera is not connected!"); 
     exit();
     return;
  }
  context.enableRGB();
  cam = createImage(context.depthWidth(), context.depthHeight(), RGB);  
  maskImage = createImage(640,480,ALPHA);
  skinImage = createImage(context.depthWidth(), context.depthHeight(), RGB);
  
  
  // enable skeleton generation for all joints
  context.enableUser(SimpleOpenNI.SKEL_PROFILE_ALL);
  

  stroke(0,0,255);
  strokeWeight(3);
  smooth(); 
  size(context.depthWidth()*2, context.depthHeight()); 
  println(context.depthWidth() + " X " +context.depthHeight());
  
   
  
}

void draw()
{
  background(255);
  // update the cam
  context.update(); 
  cam = context.rgbImage();
   int[] userList = context.getUsers();
  // draw the skeleton if it's available
   
  image(cam,0,0); 
  
  for(int i=0;i<userList.length;i++)
  {
    if(context.isTrackingSkeleton(userList[i]))
        { 
          determineUserJoints(userList[i]);
          
          maskImage.loadPixels();
          skinImage.loadPixels();
          
          updateUserPixcels(userList[i]);
          
          maskImage.updatePixels();
          skinImage.updatePixels();
        }
  } 
  
  boolean useMask = false;
  if(useMask)
    {
     cam.mask(maskImage);
     cam.mask(skinImage);   
     image(cam, 640 ,0);
    }
  else
   {
     image(maskImage,640,0);
     image(skinImage,0,0);
   }  
  
   
  
  line(0, startY , 640, startY); 
  
  
}


private void updateUserPixcels(int userId)
 { 
       
 
     int[] userPixels = context.getUsersPixels(userId); 
      
     
     int yOffset = 640*40; 
     
     PVector neck = neckPos.get(userId);
   
    // for each row in the depth image
    for(int y = 0; y < 480; y++)
    {
        
      //if(y<neck.y)//continue;
      
      
      // look at each pixel in the row
      for(int x = 0; x <640; x++)
      {
        // pull out the corresponding value from the depth array
        int i = x + (y * 640);
        int di = i+yOffset;
        if(di>=640*480)break;
        
        
        if(i<yOffset)
          { maskImage.pixels[di] = color(255,255,255); //ignore upper pixcel from depth camera
            skinImage.pixels[di] = color(0,0,0); //ignore upper pixcel from depth camera
           continue;
          }
          
        if(y<neck.y-10)  
          {maskImage.pixels[di] = color(255,255,255); //ignore if upper than neck - 10 
           continue;
          }
        
       
        
        if(userPixels[i]!=0)
         { maskImage.pixels[di] = color(0,0,0);
           skinImage.pixels[di] = color(255,0,0);
         }
        else
         {
          maskImage.pixels[di] =  color(255,255,255) ; 
          skinImage.pixels[di] =  color(0,0,0);
         }
          
         
                
        
      }//for x  
    }//for y
     
 } 
 
  
void determineUserJoints(int userId)
{
  // to get the 3d joint data
  
  PVector jointPos  = new PVector();
  PVector converted = new PVector();
  
  context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_NECK,jointPos);
  context.convertRealWorldToProjective(jointPos, converted); 
  neckPos.put(userId, converted);
}

// -----------------------------------------------------------------
// SimpleOpenNI events

void onNewUser(int userId)
{
  println("onNewUser - userId: " + userId);
  println("  start pose detection");
  
  if(autoCalib)
    context.requestCalibrationSkeleton(userId,true);
  else    
    context.startPoseDetection("Psi",userId);
}

void onLostUser(int userId)
{
  println("onLostUser - userId: " + userId);
}

void onExitUser(int userId)
{
  println("onExitUser - userId: " + userId);
}

void onReEnterUser(int userId)
{
  println("onReEnterUser - userId: " + userId);
}

void onStartCalibration(int userId)
{
  println("onStartCalibration - userId: " + userId);
}

void onEndCalibration(int userId, boolean successfull)
{
  println("onEndCalibration - userId: " + userId + ", successfull: " + successfull);
  
  if (successfull) 
  { 
    println("  User calibrated !!!");
    context.startTrackingSkeleton(userId); 
  } 
  else 
  { 
    println("  Failed to calibrate user !!!");
    println("  Start pose detection");
    context.startPoseDetection("Psi",userId);
  }
}

void onStartPose(String pose,int userId)
{
  println("onStartPose - userId: " + userId + ", pose: " + pose);
  println(" stop pose detection");
  
  context.stopPoseDetection(userId); 
  context.requestCalibrationSkeleton(userId, true);
 
}

void onEndPose(String pose,int userId)
{
  println("onEndPose - userId: " + userId + ", pose: " + pose);
}

