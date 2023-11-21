// Initial version 12-Nov-2023, Dean Gienger

//------------------------------------------------------
// File System routines
//----------------------------------------------------------
// File system support routines for SPIFFS or SD_MMC file system
//
// requires something like: fs::FS & fileSystem = SD_MMC;
//  Use Sketch | Add file to add the file from the Components folder in the Arduino IDE
//  Use #include "FileSystemService.h" in the main folder


void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
   zprint("Listing directory: ");
   zprintln(dirname);

   File root = fs.open(dirname);
   if(!root){
      zprintln("- failed to open directory");
      return;
   }
   if(!root.isDirectory()){
      zprintln("- not a directory");
      return;
   }

   File file = root.openNextFile();
   while(file){
      if(file.isDirectory()){
         zprint("  DIR : ");
         zprintln(file.name());
         if(levels){
            listDir(fs, file.name(), levels -1);
         }
      } else {
         zprint("  FILE: "); zprint(file.name()); zprint("\tSIZE: "); zprintln(file.size());
      }
      file = root.openNextFile();
   }
}

void readFile(fs::FS &fs, const char * path){
   char buf[2];
  
   zprint("Reading file: "); zprintln(path);

   File file = fs.open(path);
   if(!file || file.isDirectory()){
       zprintln("- failed to open file for reading");
       return;
   }

   zprintln("-------- data read from file --------");
   buf[1]='\0';
   while(file.available()){
      buf[0]=file.read();
      zprint(buf);
   }
   file.close();
}

void copyFile(fs::FS &fs, const char * srcpath, const char * destpath){
   zprint("Copy file: "); zprint(srcpath); zprint(" to "); zprintln(destpath);
 
   File inpfile = fs.open(srcpath);
   if(!inpfile || inpfile.isDirectory()){
       zprintln("- failed to open file for reading");
       return;
   }
  
   File outfile = fs.open(destpath, FILE_WRITE);
   if(!outfile){
      zprintln("- failed to open file for writing");
      return;
   }

   while (inpfile.available())
   {
      outfile.write(inpfile.read());
   }
   inpfile.close();
   outfile.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
   zprint("Writing file: "); zprintln(path);

   File file = fs.open(path, FILE_WRITE);
   if(!file){
      zprintln("- failed to open file for writing");
      return;
   }
   if(file.print(message)){
      zprintln("- file written");
   }else {
      zprintln("- file write failed");
   }
}

void appendFile(fs::FS &fs, const char * path, const char * message){
   zprint("Appending to file: "); zprintln(path);

   File file = fs.open(path, FILE_APPEND);
   if(!file){
      zprintln("- failed to open file for appending");
      return;
   }
   if(file.println(message)){
      zprintln("- message appended");
   } else {
      zprintln("- append failed");
   }
   file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
   zprint("Renaming file "); zprint(path1); zprint(" to "); zprintln(path2);
   if (fs.rename(path1, path2)) {
      zprintln("- file renamed");
   } else {
      zprintln("- rename failed");
   }
}

void deleteFile(fs::FS &fs, const char * path){
   zprint("Deleting file: "); zprintln(path);
   if(fs.remove(path)){
      zprintln("- file deleted");
   } else {
      zprintln("- delete failed");
   }
}


//----------------------------------------------------------
// read line from input text file
int readln(File finp, uint8_t* buf, int maxlen)
{
  // return true on successful read, false on EOF
  // 10 or 13 (LF, CR) or both are EOL indicators
  int len=0;
  int eof=false;

  buf[0]=0;
  while (len<(maxlen-1))
  {
    if (!finp.available())
    {
      eof=true;
      break;
    }
    char c = finp.read();
    if (c < 0) 
    {
      eof=true;
      break; // EOF
    }
    if (c==13) continue; // ignore CR
    if (c==10) break; // end-of-line
    buf[len++]=c;
  }
  buf[len]=0; // null terminate
  return !eof;
}

//----------------------------------------------------------
// retrieve a value for a key in the config file
int readKey(char* configFn, char* key, char* outbuf, int maxlen)
{
  // return true on success
  int retval = false; // error
  outbuf[0] = 0; // returning null string on error 
  //
  // Config file is key=value format
  // SSID=mywifi
  // PASSWORD=mypassword
  // TIMEZONE=-8
  // OFFSET=123590 
  //
  // pass in key with trailing = sign!!!

  //Serial.print("Looking for config key "); Serial.println(key);
  
  File finp = fileSystem.open(configFn, FILE_READ);
  if (!finp)
  {
    Serial.println("Unable to read config file - readKey");
    return retval;
  }
  // scan file and look for key
  char buf[128];
  int n = strlen(key);
  while (readln(finp, (uint8_t*) buf, 127))
  {
    if (strncmp(buf,key,n) == 0) // found
    {
      //Serial.print("Found key "); Serial.println(buf);
      strncpy(outbuf,&buf[n],maxlen);
      //Serial.println(outbuf);
      retval = true;
      break;
    }
  }
  finp.close();
  if (!retval)
  {
    Serial.println("Unable to find key");
    Serial.println(key);
  }
  else
  {
    Serial.print("Found key ");
    Serial.print(key);
    Serial.println(outbuf);
  }
  return retval; 
}
