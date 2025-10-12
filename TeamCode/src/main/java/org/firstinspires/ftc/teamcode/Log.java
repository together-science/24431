package org.firstinspires.ftc.teamcode;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;


/** A simple way to log data to a file */

public class Log {
    private static final String BASE_FOLDER_NAME = "FIRST";
    private Writer fileWriter;
    private String line;
    private boolean logTime;
    private long startTime;
    private boolean disabled = false;
    //private String fileName ="data";

       
    Log(String fileName,boolean logTime){
    //   if (logTime) startTime = System.nanoTime();
    //   this.logTime = logTime;
       //String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
       String directoryPath = "/sdcard/FIRST/" + BASE_FOLDER_NAME;
       File directory = new File(directoryPath);
       //no inspection ResultOfMethodCallIgnored
      
       if (!directory.isDirectory())
       {
           directory.mkdir();
           //if(!fDir.mkdir()){
           //    DbgLog.error("Could not make directory + directoryPath");
           //}
       }
       
       try{
           fileWriter = new FileWriter(directoryPath+"/"+fileName+".txt");
       } catch (IOException e){
           e.printStackTrace();
       }
    }   
       public boolean isDisable(){
           return disabled;
       }
       
       public void setDisabled(boolean disable){
           this.disabled = disabled;
       }
       
       public void close(){
           try{
               fileWriter.close();
           } catch (IOException e){
               e.printStackTrace();
           }
           
       }
    
    public void update() {
        if (disabled) return;
        try {
              line = "TEST sentence";
              fileWriter.write(line+"\n");
              line = "";
            } catch (IOException e) {
                e.printStackTrace();
        }
        
    }
    
    public void addData(String data) {
        if (disabled) return;
        if (!line.equals("")) line += ",";
        line += data;
    }
    
    public void addData(Object data){
       addData(data.toString()); 
    }
    public void addData(boolean data){
       addData(String.valueOf(data)); 
    }
        public void addData(byte data){
       addData(String.valueOf(data)); 
    }
        public void addData(char data){
       addData(String.valueOf(data)); 
    }
        public void addData(short data){
       addData(String.valueOf(data)); 
    }
        public void addData(int data){
       addData(String.valueOf(data)); 
    }
        public void addData(long data){
       addData(String.valueOf(data)); 
    }
        public void addData(float data){
       addData(String.valueOf(data)); 
    }
        public void addData(double data){
       addData(String.valueOf(data)); 
    }
   
    
    // todo: write your code here
}