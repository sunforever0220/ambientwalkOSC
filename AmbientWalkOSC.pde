//  **Be sure that your Sketch has activated these permissions:
//    INTERNET
//    RECORD_AUDIO
//    WRITE_EXTERNAL_STORAGE
//    WAKE_LOCK

//Two modes:data logging or send OSC to computer
import android.app.Activity;//org
import android.content.Context;//org
import android.os.Bundle;//org
import android.os.PowerManager;//org
//sensor libraries
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
//sound libraries:audiorecord
import android.media.AudioFormat;
import android.media.AudioRecord;
import android.media.MediaRecorder;
import android.media.AudioTrack;
//tone generator
import android.media.ToneGenerator;
import android.media.AudioManager;
//OSC
import oscP5.*;
import netP5.*;
import controlP5.*;
import android.net.wifi.WifiManager;
import android.text.format.Formatter;
//GPS
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.location.Criteria;
import java.util.Locale;                     // note these is a Java import, not Android
import java.util.List;

//OSC settings
OscP5 oscP5;
NetAddress myRemoteLocation;
    //Parameter initialisation: sensor parameter
    SensorManager sensorManager;       
    SensorListener sensorListener; 
    //--step counting with pedometer (for API 19+)
    /*Sensor pedometer; 
    float step, isStep, prev_step, abs_stepcount;
    */
    //--step counting with accelerometer (for API 16)--
    Sensor accelerometer;  
    float[] accelData; 
    //list of acc-magnitude values over time (of a window of 10--1sec)
    FloatList acc_magnitude;
    float magnitude,acc_sum,acc_avg;
    double pace;
    //totalcount, stepcount and windowsize for counting average
    int stepcount=0,windowsize=10,peak_index,prev_index,startframe=0;
    
    //--flag whether user tapped the screen
    boolean button_clicked=false;
    //audio vars (breath recording)
    short[] buffer = null;                                    
    AudioRecord audioRecord = null;
    AudioTrack audioTrack = null;
    int bufferSize=320;
    int bufferReadResult;
    float[] volume = null;float maxvol=0;
    int buflen;
//vars for finding peak of breath sound
    int current_peak=0,prev_peak=0;
    float period;
    //generate sound of a certain pitch          
    int duration = 10; // seconds
    int gentonefreq = 8000;
    int numSamples = duration * gentonefreq;
    double sample[]=null;
    byte generatedSnd[]=null;
    double freqOfTone_breath,freqOfTone_step; // hz (variable)
    
   //offset of default frequency:c major triad
    double offset1,offset2,offset3;
//wakelock
PowerManager pm;
PowerManager.WakeLock wl;
//data logging
PrintWriter OUTPUT;
//initialscreen images
PImage title,instruction,buttonimg,dmlogo;
//WifiManager setting (to get ip address of the device)
int start_time;
WifiManager wm;
String ip="";
String isOSC="false";
//GPS
double longitude, latitude;

 
void setup() 
{       
        
        frameRate(10);//50 frame per sec
        smooth();
        //load images to be placed at front screen
        title = loadImage("title_logo.png");
        instruction = loadImage("instruction.png");
        buttonimg=loadImage("button.png");
        dmlogo=loadImage("dmlogo.png");
        //accelerometer setting
        acc_magnitude=new FloatList();//refresh magnitude list every frame
        prev_index=0;
        offset1=261.63;//Pitch:C4
        offset2=329.63;//Pitch:E4
        offset3=392.00;//Pitch:G4
        //audio setup
        int freq =8000;
        int chan = AudioFormat.CHANNEL_IN_MONO;
        int enc  = AudioFormat.ENCODING_PCM_16BIT;
        int src  = MediaRecorder.AudioSource.MIC;
        buflen = AudioRecord.getMinBufferSize(freq, chan, enc);
       //println("getMinBufferSize: "+buflen);
        audioRecord = new AudioRecord(src,freq,chan,enc,buflen);//record every 100ms
        audioRecord.startRecording();
        buffer = new short[bufferSize];
        volume = new float[bufferSize];
 //wakelock setting       
pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
wl = pm.newWakeLock(PowerManager.SCREEN_DIM_WAKE_LOCK, "My Tag");
wl.acquire();
//OSC setting:localhost
//myRemoteLocation = new NetAddress("127.0.0.1",12000);
//OSC setting:iphone5 (need to be configured again)
myRemoteLocation = new NetAddress("192.168.0.100",12000);//BHCI wifi ip
//myRemoteLocation = new NetAddress("172.20.10.6",12000);

//ip=NetInfo.getHostAddress();
wm = (WifiManager) getSystemService(WIFI_SERVICE);
ip = Formatter.formatIpAddress(wm.getConnectionInfo().getIpAddress());
//println(ip);

}
 
void draw() //update per frame, stop working when screen locked
{
  
  background(50,10,10);
  //paint initial screen with start button
  paintfirstscreen();
  
  //if button clicked:
  if(button_clicked){
   background(50,10,10);
    isOSC="true";
     
       try {
    fill(255);noStroke();
   textSize(30);   
text("Breath Period: " + period+"s", width/10, height*4/5); 
  text("Number of Steps: "+stepcount, width/10,height*5/6);
      bufferReadResult = audioRecord.read(buffer, 0, bufferSize);
     
   thread("breathRecording");
   for (int i=0;i<bufferReadResult-51;i+=50){
             stroke(50,10,10);
          fill(100,100,100,volume[i]*4);         
 ellipse(width/2,height/2,volume[i]*4,volume[i]*4);
 }
 //text("Volume: "+maxvol,width/10,height*6/7);
           //part 2:step detection with accelerometer||android 4.1.2+ (API16-19)
           //get sensor data
   thread("stepDetection");

           //generate the tones based on two data
           if(frameCount%20==1){
           genTone(freqOfTone_breath,freqOfTone_step);
           playSound(); 
       }
    
        
       //Beats per sec (training data for steps)
       
     //  ToneGenerator tg = new ToneGenerator(AudioManager.STREAM_NOTIFICATION, 100);
     //  tg.startTone(ToneGenerator.TONE_PROP_BEEP);
    
       //send OSC per sec: breath period, walking pace, ratio
       if (frameCount%10==1){
   export2Txt(period, pace);
   getGPSLocation();
   OscMessage myOscMessage = new OscMessage("/data");

   myOscMessage.add(isOSC);
   myOscMessage.add(ip);
   myOscMessage.add(period);
   //OscP5.flush(myOscMessage1,myRemoteLocation);
   //OscMessage myOscMessage2 = new OscMessage("/walk_pace");
   myOscMessage.add(pace);
   myOscMessage.add(maxvol);
   myOscMessage.add(latitude);
   myOscMessage.add(longitude);
   
   //OscP5.send(myOscMessage, myRemoteLocation); 
   OscP5.flush(myOscMessage,myRemoteLocation);
 maxvol=0;
   /*
   OscMessage myOscMessage3 = new OscMessage("/ratio");
   myOscMessage1.add(ratio);
   OscP5.flush(myOscMessage3,myRemoteLocation);
   */
  
  //end sending OSC
  
       }
     // end if frameCount%10=1
     if(millis()-start_time>=1000||millis()-start_time==0){
     ToneGenerator tg = new ToneGenerator(AudioManager.STREAM_MUSIC, 60);
     tg.startTone(ToneGenerator.TONE_PROP_BEEP);
     start_time=millis();
    }
       
    }//end try
    catch (IllegalArgumentException e) 
      {
        freqOfTone_breath=offset2;
        freqOfTone_step=2*offset3;
        genTone(freqOfTone_breath,freqOfTone_step);
        e.printStackTrace();
      } 
    catch (IllegalStateException e) 
      {
        freqOfTone_breath=2*offset1;
        freqOfTone_step=3*offset2;
        genTone(freqOfTone_breath,freqOfTone_step);
        e.printStackTrace();
      } 
    catch (RuntimeException e) 
      {
        freqOfTone_breath=4*offset1;
        freqOfTone_step=offset3;
        genTone(freqOfTone_breath,freqOfTone_step);
        e.printStackTrace();
      } 

  }//end button clicked
} //end draw 

   public void breathRecording(){

   //part 1: breath recording (per second)
  
       bufferReadResult = audioRecord.read(buffer, 0, bufferSize);
  
      //println("bufferReadResult: "+bufferReadResult);
     // println("real buffer length: "+buffer.length);
       maxvol = 0;//reset volume for each frame
     
        //println("Time: "+second()+" Frame: "+frameCount+"||Bufferdata: ");
       for(int i=0;i<bufferReadResult-1;i++){//down sampling to 800 samples/sec
 //println(i+": "+buffer[i]+"");
         if(Math.abs(buffer[i])==0){volume[i]=0;}
           else{
           volume[i]=20*log(Math.abs(buffer[i])/20)/log(10);//transfer to decibel
           }
           
           maxvol=Math.max(maxvol,volume[i]); 

           //real-time debouncing
            if(i>=2&&i<=bufferReadResult-1){
              if (volume[i-2]<volume[i-1]&&volume[i-1]>volume[i]&&volume[i-1]>=60){
                          //debouncing algorithm: only indicate foundmax when fulfilling (either of) these two criteria
                          current_peak=(frameCount-1)*320+i-1;//get absolute sample index of the maximum
          //need further editing****
          if(prev_peak==0){prev_peak=current_peak;}
          else if(current_peak-prev_peak>=320*10){
            period=(current_peak-prev_peak)*2/(320*10);
            prev_peak=current_peak;
          }  
          //println(prev_peak);
          //else:do nothing if found a point within debouncing period
        } //else: do nothing if not found a maximum
     }//end debouncing
   }//end for loop (transfer into decibel volume)
           //draw volume
            //fill(50,50,50,maxvol);
 //ellipse(width/2,height/2,maxvol,maxvol);
        // fill(255);
       //text("Breath Period: " + period+"s", width/10, height*4/5); 
       //calculate breath rate
       if(period!=0){
     freqOfTone_breath=(double)1/period*10*offset1;
       }
       else{freqOfTone_breath=2*offset1;}
  
    }//end function:breath recording
    
    public void stepDetection(){
   if (accelData != null) {
     magnitude=sqrt(accelData[0]*accelData[0]+accelData[1]*accelData[1]+accelData[2]*accelData[2]);
     //test accelerometer data logging (need to be commented for real app)
     // OUTPUT.println(magnitude);  // here we export the coordinates of the vector using String concatenation!
     //text("Acceleration: "+magnitude, width/10,height*7/8);
     acc_magnitude.append(magnitude);//append new value at the end
    // acc_sum=acc_sum+magnitude;//add new value
     //see whether to subtract the first value
if(frameCount%windowsize==0){
  //acc_avg=acc_sum/windowsize;
//find peak within this range
for(int i=1;i<windowsize-1;i++){
//peak detection with debouncing
peak_index=detect_peak(prev_index, i-1,i,i+1);

}//end for (peak detection)
acc_magnitude=new FloatList();
}//end if (framecount=windowsize)

 if(peak_index!=prev_index){
 stepcount++;
 if(stepcount==1){startframe=frameCount;}
 prev_index=peak_index;
  }
}//end if accelData!=null

       //generate a tone according to steps per 1 sec (10 frames)
       if(frameCount%10==1){       
    if(stepcount!=0&&frameCount!=1){
      pace=(double)stepcount*10/(frameCount-startframe);
              freqOfTone_step=pace*offset1;
        }
       else{freqOfTone_step=random(220,2000);}
       }//end frameCount%10==1
    }//end function:step detection
    
void stop() 
{ //audiorecord
  audioRecord.stop();
  audioRecord.release();
  audioRecord = null;
  //audiotrack
  audioTrack.flush();
  audioTrack.stop();
  audioTrack.release();
  audioTrack = null; 
  wl.release();
  OUTPUT.close();
  isOSC="false";
  OscMessage myOscMessage = new OscMessage("/data");
   myOscMessage.add(isOSC);
  OscP5.flush(myOscMessage,myRemoteLocation);
}
 
void mousePressed()
{
  button_clicked=true;
  //for stepcounter (API 19+)
//abs_stepcount=0;
//println(abs_stepcount);//check step count
frameCount=0;
start_time=millis();
//start data logging (to sdcard)
//data logging for breath period and walking pace
OUTPUT = createWriter("/sdcard/ambientwalk_data_"+month()+"_"+day()+"_"+hour()+"_"+minute()+"_"+second()+".txt");
OUTPUT.println("FrameCount"+","+"Breath Period (sec)"+","+"Walking Pace (Hz)");
//OUTPUT = createWriter("/sdcard/breath_rec"+hour()+"_"+minute()+"_"+second()+".txt");

}

void onResume() //register sensor listener
{
  super.onResume();
  sensorManager = (SensorManager)getSystemService(Context.SENSOR_SERVICE);
  sensorListener = new SensorListener();
  //Step counter for API 19+/Nexus5
  /*pedometer = sensorManager.getDefaultSensor(Sensor.TYPE_STEP_COUNTER);
 sensorManager.registerListener(sensorListener, pedometer, SensorManager.SENSOR_DELAY_FASTEST); 
*/
//Use accelerometer for API 16+
accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
  sensorManager.registerListener(sensorListener, accelerometer, SensorManager.SENSOR_DELAY_FASTEST); 
//isOSC="true";
}

void onPause() //unregister sensor listener
{
  sensorManager.unregisterListener(sensorListener);
  super.onPause();
  //close data logging
   //OUTPUT.flush();
 //OUTPUT.close();
 isOSC="false";
 OscMessage myOscMessage = new OscMessage("/data");
   myOscMessage.add(isOSC);
  OscP5.flush(myOscMessage,myRemoteLocation);
}

//peak detection (with indices as input), returns
int detect_peak(int prev_peak,int prev, int current, int next){
   // println(acc_magnitude.get(current));
   if(acc_magnitude.get(current)>acc_magnitude.get(prev)&&acc_magnitude.get(current)>acc_magnitude.get(next)&&acc_magnitude.get(current)>1.3&&acc_magnitude.get(current)<4){
   peak_index=10*floor(frameCount/10)+current;
   if(peak_index-prev_peak<5){peak_index=prev_peak;}
   } 
  return peak_index;  
 }
 
//generate stereo tones (left:tone 1, right:tone 2)
void genTone(double freqOfTone_1, double freqOfTone_2)
 {
    sample=new double[numSamples];
   generatedSnd=new byte[2*numSamples];
   // fill out the array
   for (int i = 0; i < numSamples; i++) {
     if(i%4==0){
       //sample[i] = 4*(Math.sin(2 * Math.PI * i / (gentonefreq/freqOfTone_1))+Math.sin(6 * Math.PI * i / (gentonefreq/freqOfTone_1))/3+Math.sin(10 * Math.PI * i / (gentonefreq/freqOfTone_1))/5)/Math.PI;
     sample[i] = 0.2*(Math.sin(2 * Math.PI * i / (gentonefreq/freqOfTone_1))+Math.sin(2 * Math.PI * i / (gentonefreq/freqOfTone_2)));
   }
 else if (i%4==1){
   sample[i]=0.3*(Math.sin(2 * Math.PI * i / (gentonefreq/offset1))+Math.sin(2 * Math.PI * i / (gentonefreq/offset2))+Math.sin(2 * Math.PI * i / (gentonefreq/offset3)));
   
 }
   else if(i%4==2){
       //sample[i] = 4*(Math.sin(2 * Math.PI * i / (gentonefreq/freqOfTone_1))+Math.sin(6 * Math.PI * i / (gentonefreq/freqOfTone_1))/3+Math.sin(10 * Math.PI * i / (gentonefreq/freqOfTone_1))/5)/Math.PI;
     sample[i] = 0.05*Math.sin(2 * Math.PI * i / (gentonefreq/freqOfTone_1));
   }
   else {
     sample[i] = 0.05*Math.sin(2 * Math.PI * i / (gentonefreq/freqOfTone_2));
   }
}
        // convert to 16 bit pcm sound array
        // assumes the sample buffer is normalised.
   int idx = 0;
   for (final double dVal : sample) {
       // scale to maximum amplitude
       final short val = (short) ((dVal * 32767));
       // in 16 bit wav PCM, first byte is the low order byte
       generatedSnd[idx++] = (byte) (val & 0x00ff);
       generatedSnd[idx++] = (byte) ((val & 0xff00) >>> 8);
    }
 }
 
void playSound()
{
       audioTrack = new AudioTrack(AudioManager.STREAM_MUSIC,
       gentonefreq, AudioFormat.CHANNEL_OUT_STEREO,
       AudioFormat.ENCODING_PCM_16BIT, generatedSnd.length,
       AudioTrack.MODE_STATIC);
       audioTrack.write(generatedSnd, 0, generatedSnd.length);
       audioTrack.play();
}

 
 //paint first screen
 void paintfirstscreen()
 {
   background(50,10,10);
   //fill the bottom 1/3 with darker color
   if(title.width>=width){
   image(title, 0,height/3,width,title.height*width/title.width);
   }
  else{image(title, (width-title.width)/2,height/3);}
   if(instruction.width>=width){
     image(instruction, 0,height*3/5+buttonimg.height*width/buttonimg.width/2, width,instruction.height*width/instruction.width);
   }
   else{image(instruction, (width-instruction.width)/2,height*3/5+buttonimg.height);}
   if(buttonimg.width>=width){
     image(buttonimg,0,height/2, width, buttonimg.height*width/buttonimg.width);
   }
   else{image(buttonimg,(width-buttonimg.width)/2,height*5/9);}
   if(dmlogo.width>=width/2){
   image(dmlogo,width/2,height-dmlogo.height*width/2/dmlogo.width, width/2, dmlogo.height*width/2/dmlogo.width);
   }
   else{image(dmlogo,width-dmlogo.width,height-dmlogo.height);}

}

//data logging to sd card

void export2Txt(float breath, double pace){
  
 
    OUTPUT.println(frameCount+","+breath+","+pace);  // here we export the coordinates of the vector using String concatenation!
  
 OUTPUT.flush();
  println("data has been exported");
}

// function to load the location
void getGPSLocation() {
  // if GPS can be accessed, store in 'location' string
  try {
    // get GPS coords - a lot of steps, but basically we load the best provider (either 3G or
    // GPS) and get the current location!
    LocationManager lm = (LocationManager)getSystemService(Context.LOCATION_SERVICE);
    Criteria criteria = new Criteria();                          // default provider criteria
    String provider = lm.getBestProvider(criteria, false);       // can get a list of providers, or the best
    Location gpsLocation = lm.getLastKnownLocation(provider);    // location (last known if new isn't available)

    // parse coordinates from Location
    longitude = gpsLocation.getLongitude();
    latitude = gpsLocation.getLatitude();
    println("x: "+latitude+" y: "+longitude);
  }catch (NullPointerException npe) {
    println("GPS not available");
  }
  // problem loading address
  
}//end getGPSLocation

class SensorListener implements SensorEventListener 
{
  void onSensorChanged(SensorEvent event) 
  {//if step counter is available
  /*
    if (event.sensor.getType() == Sensor.TYPE_STEP_COUNTER)    {
      step = event.values[0];
    }
    */
    //if accelerometer only
    if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) 
    {
      accelData = event.values;
    }
   
  }
  void onAccuracyChanged(Sensor sensor, int accuracy) 
  {
       //todo 
  }
}



