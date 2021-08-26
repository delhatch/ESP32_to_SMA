/*
 NANODE SMA PV MONITOR
 SMANetArduino.ino
 */

unsigned int readLevel1PacketFromBluetoothStream(int index) {
  unsigned int retcmdcode;
  bool errorCodePCS;
  bool errorCodeVSA;
  bool errorCodePIFM;
  //HowMuchMemory();

  //Serial.print(F("#Read cmd="));
  //Serial.print("READ=");  Serial.println(index,HEX);
  retcmdcode = 0;
  errorCodePCS = false;
  errorCodeVSA = false;
  errorCodePIFM = false;
  //level1packet={0};

  //Wait for a start packet byte
  while (getByte() != '\x7e') { 
    delay(1);  
  }

  byte len1 = getByte();
  byte len2 = getByte();
  byte packetchecksum = getByte();

  if ( (0x7e ^ len1 ^ len2) == packetchecksum ) errorCodePCS = false;
  else {
    debugMsgln("Inv hdr");
    Serial.println("*** Invalid header chksum.");
    errorCodePCS = true;
    return 0xFFFF;
  }
  
  for (int i=0; i<6; i++)  Level1SrcAdd[i]=getByte();
  for (int i=0; i<6; i++)  Level1DestAdd[i]=getByte();

  retcmdcode = getByte()+( getByte() * 256 );
  packetlength = len1 + (len2*256);
  //Serial.println(" ");
  //Serial.print("packetlength = ");
  //Serial.println(packetlength);
  //if (packetlength > 120) packetlength = 120;

  if ( ValidateSenderAddress() ) {
    //Serial.println("*** Message is from SMA.");
    errorCodeVSA = false;
  }
  else {
    debugMsgln("P wrng dest");
    //Serial.println("*** Not from SMA.");
    errorCodeVSA = true;
  }
  if ( IsPacketForMe() ) {
    //Serial.println("*** Message is to ESP32.");
    errorCodePIFM = false;
  }
  else {
    debugMsgln("P wrng snder");
    //Serial.println("*** Not to me.");
    errorCodePIFM = true;
  }
        
  //If retcmdcode==8 then this is part of a multi packet response which we need to buffer up
  //Read the whole packet into buffer
  //=0;  //Seperate index as we exclude bytes as we move through the buffer

  //Serial.println(" ");
  //Serial.print( "packetlength-level1headerlength = " );
  //Serial.println( packetlength-level1headerlength );
  for(int i=0; i<(packetlength-level1headerlength); i++) {
    //Serial.print( " index = " );
    //Serial.print( index );
    //Serial.print( " " );
    level1packet[index] = getByte();

    //Keep 1st byte raw unescaped 0x7e
    //if (i>0) {
    if ( escapenextbyte == true ) {
      level1packet[index] = level1packet[index] ^ 0x20;
      escapenextbyte = false;         
      index++;
    } 
    else {
      if ( (level1packet[index] == 0x7d) ){
        //Throw away the 0x7d byte
        escapenextbyte = true;
      } 
      else index++;
    }
  } // End of for loop that reads the ENTIRE packet into level1packet[]

  //Reset packetlength based on true length of packet
  packetlength = index + level1headerlength;  // Length of the entire packet received. Which is 18 greater
  //                                               than the amount of data actually added to level1packet[]
  packetposition = index;       // Index into level1packet[] of the first byte after the Level 1 header stuff.
  //                        The byte at level1packet[index] will be 0x7E for a level 2 packet. NOT 0x7E for a level 1 packet.

  if( errorCodePCS || errorCodeVSA || errorCodePIFM ) {
    //Serial.println(" ");
    //Serial.println("readLevel1PacketFromBluetoothStream now returning with return code = 0xFFFF.");
    return 0xFFFF;
  }
  else {
    //Serial.println(" ");
    //Serial.print("readLevel1PacketFromBluetoothStream now returning with return code = ");
    //Serial.println(retcmdcode);
    return retcmdcode;
  }
}
//-----------------------------------------------------------
void prepareToReceive(){
  FCSChecksum=0xFFFF;
  packetposition=0;
  escapenextbyte=false;
}
//------------------------------------------------------------
bool containsLevel2Packet() {
  //Serial.println(" ");
  //Serial.print("packetlength = ");
  //Serial.println(packetlength);
  //Serial.print("level1headerlength = ");
  //Serial.println(level1headerlength);
  if ((packetlength-level1headerlength) < 5) return false;

  return (level1packet[0] == 0x7e &&
    level1packet[1] == 0xff &&
    level1packet[2] == 0x03 &&
    level1packet[3] == 0x60 &&
    level1packet[4] == 0x65);
}
//---------------------------------------------------------------
// This function will read BT packets from the inverter until it finds one that is addressed to me, and
//   has the same command code as the one I am waiting for = cmdcodetowait.
// It is called when you know a Level1 message is coming.
// NOTE: NEVER call this function with cmdcodetowait = 0xFFFF. That value is returned
//   if the packet has a checksum error, or it's not addressed to me, etc...
// This function just discards the packet if there is an error, and resumes waiting.

void waitForPacket(unsigned int cmdcodetowait) {

  prepareToReceive();

  do {
    cmdcode = readLevel1PacketFromBluetoothStream(0);
  } while ( cmdcode != cmdcodetowait);

  return;
}
//------------------------------------------------------------------
// This code will also look for the proper command code.
// The difference to the function above is that it will search deeply through the level1packet[] array.
// Level 2 messages (PPP messages) follow the Level 1 packet header part. Each Level 2 packet starts with
//   7E FF 03 60 65.
void waitForMultiPacket(unsigned int cmdcodetowait) {
  //debugMsg("waitForMultiPacket=0x");  Serial.print(cmdcodetowait,HEX);
  
  //prepareToReceive();
  //Serial.println(" ");
  while ( cmdcode != cmdcodetowait ) {
    prepareToReceive();
    //Serial.println("*** In waitForMultiPacket. About to call readLevel1PacketFromBluetoothStream(0) ***");
    //cmdcode = readLevel1PacketFromBluetoothStream(packetposition);
    cmdcode = readLevel1PacketFromBluetoothStream(0);
    //Serial.println(" ");
    //Serial.print("*** Done reading packet. cmdcode = ");
    //Serial.println(cmdcode);
  }
  //Serial.println("Now returning from 'waitForMultiPacket'.");
  return;
}

void writeSMANET2Long(unsigned char *btbuffer,unsigned long v) {
  writeSMANET2SingleByte(btbuffer,(unsigned char)((v >> 0) & 0XFF));
  writeSMANET2SingleByte(btbuffer,(unsigned char)((v >> 8) & 0XFF));
  writeSMANET2SingleByte(btbuffer,(unsigned char)((v >> 16) & 0xFF)) ;
  writeSMANET2SingleByte(btbuffer,(unsigned char)((v >> 24) & 0xFF)) ;
}

void writeSMANET2uint(unsigned char *btbuffer,unsigned int v) {
  writeSMANET2SingleByte(btbuffer,(unsigned char)((v >> 0) & 0XFF));
  writeSMANET2SingleByte(btbuffer,(unsigned char)((v >> 8) & 0XFF));
  //writeSMANET2SingleByte(btbuffer,(unsigned char)((v >> 16) & 0xFF)) ;
  //writeSMANET2SingleByte(btbuffer,(unsigned char)((v >> 24) & 0xFF)) ;
}

/*
void displaySpotValues(int gap) {
  unsigned long value = 0;
  unsigned int valuetype=0;
  for(int i=40+1;i<packetposition-3;i+=gap){
    valuetype = level1packet[i+1]+level1packet[i+2]*256;
    memcpy(&value,&level1packet[i+8],3);
    //valuetype 
    //0x2601=Total Yield Wh
    //0x2622=Day Yield Wh
    //0x462f=Feed in time (hours) /60*60
    //0x462e=Operating time (hours) /60*60
    //0x451f=DC Voltage  /100
    //0x4521=DC Current  /1000
    Serial.print(valuetype,HEX);
    Serial.print("=");
    memcpy(&datetime,&level1packet[i+4],4);
    digitalClockDisplay(datetime);
    Serial.print("=");Serial.println(value);
  }
}
*/

void writeSMANET2SingleByte(unsigned char *btbuffer, unsigned char v) {
  //Keep a rolling checksum over the payload
  FCSChecksum = (FCSChecksum >> 8) ^ pgm_read_word_near(&fcstab[(FCSChecksum ^ v) & 0xff]);

  if (v == 0x7d || v == 0x7e || v == 0x11 || v == 0x12 || v == 0x13) {
    btbuffer[packetposition ++]=0x7d;
    btbuffer[packetposition ++]=v ^ 0x20;
  } 
  else {
    btbuffer[packetposition ++]=v;
  }
}

void writeSMANET2Array(unsigned char *btbuffer, unsigned char bytes[],byte loopcount) {
  //Escape the packet if needed....
  for(int i=0;i<loopcount;i++) {
    writeSMANET2SingleByte(btbuffer,bytes[i]);
  }//end for
}

void writeSMANET2ArrayFromProgmem(unsigned char *btbuffer,prog_uchar ProgMemArray[],byte loopcount) {
    //debugMsg("writeSMANET2ArrayFromProgmem=");
    //Serial.println(loopcount);
  //Escape the packet if needed....
  for(int i=0;i<loopcount;i++) {
    writeSMANET2SingleByte(btbuffer,pgm_read_byte_near(ProgMemArray + i));
  }//end for
}


void writeSMANET2PlusPacket(unsigned char *btbuffer
,unsigned char ctrl1
,unsigned char ctrl2
,unsigned char packetcount
,unsigned char a
,unsigned char b
,unsigned char c){ 

  //This is the header for a SMANET2+ (no idea if this is the correct name!!) packet - 28 bytes in length
  lastpacketindex=packetcount;

  //Start packet
  btbuffer[packetposition++]=0x7e;  //Not included in checksum

  //Header bytes 0xFF036065
  writeSMANET2ArrayFromProgmem(btbuffer,SMANET2header,4);

  writeSMANET2SingleByte(btbuffer,ctrl1);
  writeSMANET2SingleByte(btbuffer,ctrl2);  

  writeSMANET2Array(btbuffer,sixff,6);

  writeSMANET2SingleByte(btbuffer,a);  //ArchCd
  writeSMANET2SingleByte(btbuffer,b);  //Zero

  writeSMANET2ArrayFromProgmem(btbuffer,InverterCodeArray,6);

  writeSMANET2SingleByte(btbuffer,0x00);
  writeSMANET2SingleByte(btbuffer,c);

  writeSMANET2ArrayFromProgmem(btbuffer,fourzeros,4);

  writeSMANET2SingleByte(btbuffer,packetcount);
}

void writeSMANET2PlusPacketTrailer(unsigned char *btbuffer){ 
  FCSChecksum =FCSChecksum ^ 0xffff;

  btbuffer[packetposition++]=FCSChecksum & 0x00ff;
  btbuffer[packetposition++]=(FCSChecksum >> 8) & 0x00ff;
  btbuffer[packetposition++]=0x7e;  //Trailing byte

    //Serial.print("Send Checksum=");Serial.println(FCSChecksum,HEX);
}

void writePacketHeader(unsigned char *btbuffer) {  
  writePacketHeader(btbuffer,0x01,0x00,smaBTInverterAddressArray); 
}
void writePacketHeader(unsigned char *btbuffer,unsigned char *destaddress) {  
  writePacketHeader(btbuffer,0x01,0x00,destaddress); 
}

void writePacketHeader(unsigned char *btbuffer,unsigned char cmd1,unsigned char cmd2, unsigned char *destaddress) {

  if (packet_send_counter>75) packet_send_counter=1;

  FCSChecksum=0xffff;

  packetposition=0;

  btbuffer[packetposition++]=0x7e;
  btbuffer[packetposition++]=0;  //len1
  btbuffer[packetposition++]=0;  //len2
  btbuffer[packetposition++]=0;  //checksum

  for(int i=0;i<6;i++) { 
    btbuffer[packetposition++]=myBTAddress[i];
  }

  for(int i=0;i<6;i++) { 
    btbuffer[packetposition++]=destaddress[i];
  }

  btbuffer[packetposition++]=cmd1;  //cmd byte 1
  btbuffer[packetposition++]=cmd2;  //cmd byte 2

    //cmdcode=cmd1+(cmd2*256);
  cmdcode=0xffff;  //Just set to dummy value

  //packetposition should now = 18
}

void writePacketLength(unsigned char *btbuffer) {
  //  unsigned char len1=;  //TODO: Need to populate both bytes for large packets over 256 bytes...
  //  unsigned char len2=0;  //Were only doing a single byte for now...
  btbuffer[1]=packetposition;
  btbuffer[2]=0;
  btbuffer[3]=btbuffer[0] ^ btbuffer[1] ^ btbuffer[2];  //checksum 
}

void wrongPacket() {
  Serial.println(F("*WRONG PACKET*"));  
  error(); 
}

void dumpPacket(char letter) {
  for(int i=0;i<packetposition;i++) {

    if (i % 16==0) { 
      Serial.println("");
      Serial.print(letter);
      Serial.print(i,HEX); 
      Serial.print(":");
    }

    debugPrintHexByte(level1packet[i]);
  }
  Serial.println("");
}

void debugPrintHexByte(unsigned char b) {
  if (b<16) Serial.print("0");  //leading zero if needed
  Serial.print(b,HEX);
  Serial.print(" ");
}
//-------------------------------------------------------------------------
bool validateChecksum(){
  //Checks the validity of a LEVEL2 type SMA packet

  if ( !containsLevel2Packet()  ) {
    //Wrong packet index received
    //Serial.println("*** in validateChecksum(). containsLevel2Packet() returned FALSE.");
    dumpPacket('R');

    Serial.print(F("Wrng L2 hdr"));
    return false;
  }
  //Serial.println("*** in validateChecksum(). containsLevel2Packet() returned TRUE.");
  //We should probably do this loop in the receive packet code

    if (level1packet[28-1] != lastpacketindex) 
  {
    //Wrong packet index received
    dumpPacket('R');

    Serial.print(F("Wrng Pkg count="));
    Serial.println(level1packet[28-1],HEX);
    Serial.println(lastpacketindex,HEX);
    return false;
  }

  FCSChecksum=0xffff;
  //Skip over 0x7e and 0x7e at start and end of packet
  for(int i=1;i<=packetposition-4;i++) {    
    FCSChecksum = (FCSChecksum >> 8) ^ pgm_read_word_near(&fcstab[(FCSChecksum ^ level1packet[i]) & 0xff]);
  }

  FCSChecksum =FCSChecksum ^ 0xffff;

  //if (1==1)
  if ((level1packet[packetposition-3]==(FCSChecksum & 0x00ff)) &&    (level1packet[packetposition-2]==((FCSChecksum >> 8) & 0x00ff)))
  {
    if ((level1packet[23+1]!=0) || (level1packet[24+1]!=0)) { 
      Serial.println(F("chksum err"));
      error();
    }
    return true; 
  } 
  else{
    Serial.print(F("Invalid chk="));
    Serial.println(FCSChecksum,HEX);
    dumpPacket('R');
    delay(10000);
    return false;
  }
}
//------------------------------------------------------------------------------------
/*
void InquireBlueToothSignalStrength() {
  writePacketHeader(level1packet,0x03,0x00,smaBTInverterAddressArray);
  //unsigned char a[2]= {0x05,0x00};
  writeSMANET2SingleByte(level1packet,0x05);
  writeSMANET2SingleByte(level1packet,0x00);  
  //  writePacketPayload(level1packet,a,sizeof(a));
  writePacketLength(level1packet);
  sendPacket(level1packet);
  waitForPacket(0x0004); 
  float bluetoothSignalStrength = (level1packet[4] * 100.0) / 0xff;
  Serial.print(F("BT Sig="));
  Serial.print(bluetoothSignalStrength);
  Serial.println("%");
}
*/


bool ValidateSenderAddress() {
  // Compares the SMA inverter address to the "from" address contained in the message.
  // Debug prints "P wrng dest" if there is no match.
  return (Level1SrcAdd[5]==smaBTInverterAddressArray[5] &&
    Level1SrcAdd[4]==smaBTInverterAddressArray[4] &&
    Level1SrcAdd[3]==smaBTInverterAddressArray[3] &&
    Level1SrcAdd[2]==smaBTInverterAddressArray[2] &&
    Level1SrcAdd[1]==smaBTInverterAddressArray[1] &&
    Level1SrcAdd[0]==smaBTInverterAddressArray[0]);
}

bool IsPacketForMe() { 
  // Compares the ESP32 address to the received "to" address in the message.
  // Debug prints "P wrng snder" if it does not match.
  return( ValidateDestAddress(sixzeros) ||  ValidateDestAddress(myBTAddress) || ValidateDestAddress(sixff) );
}

bool ValidateDestAddress(unsigned char *address) {
  return( Level1DestAdd[0]==address[0] &&
    Level1DestAdd[1]==address[1] &&
    Level1DestAdd[2]==address[2] &&
    Level1DestAdd[3]==address[3] &&
    Level1DestAdd[4]==address[4] &&
    Level1DestAdd[5]==address[5] );
}
