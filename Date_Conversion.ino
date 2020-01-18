/////////////////////////////////////
// Time & Date conversion
/////////////////////////////////////
struct Timestamp {
  byte hh, mm, ss, DD, MM, YY;
};      // BCD coded Time & Date info, these are updated in the time routine

struct Logvalue {                  // The way the logged values are stored
  unsigned int Dnumber;              // Day number assuming 1-1-1900 = 0
  unsigned int Snumber;              // Second number assuming 00:00:00 is 0 and 23:59:59 is 86399
  int WL;                       // measured WaterLevel in mm
};

Timestamp Tstmp1;
Timestamp Tstamp;

Logvalue LVal_1;
Logvalue LV;


long DD;
byte MM;
byte YY;

byte decToBcd(byte val){
  return (val/10*16 + val%10);
}
byte BCDTodec(byte val){
  return ((val/16)*10 + val%16);
}


void ConvertLV2TSTMP(){
  Tstmp1.hh = decToBcd((byte)(LVal_1.Snumber / 3600));
  Tstmp1.mm = decToBcd((byte)((LVal_1.Snumber % 3600) / 60));
  Tstmp1.ss = decToBcd((byte)((LVal_1.Snumber % 3600) % 60));

  DD = (LVal_1.Dnumber-36525); // # days since 01-01-2000 (a leap year)

  int YY = 00; // first assume it is 1-1-2000

    while (DD > 1461){ // if there are more than 1461 days left a full series of 4 years has passed, one of which is a leap year
    YY = YY + 4; 
    DD = DD - 1461; 
  } // DD <= 1461 

    if (DD > 1096) { // The first year of the group is a leap year with 366 days
    YY = YY + 3; 
    DD = DD - 1096; // DD <= 365
  } // DD <= 1096 


    if (DD > 731) { // The first year of the group is a leap year with 366 days
    YY = YY + 2; 
    DD = DD - 731;// DD <= 365
  } // DD <= 731

    if (DD > 366) { // The first year of the group is a leap year with 366 days
    YY = YY + 1; 
    DD = DD - 366;// DD <= 366
  } // DD <= 366

    // DD is now the day number in the year YY

  MM = 1 ; // This is the first month of the year

  if ((DD>31)&&(MM==1)){ // It is not January
    DD = DD - 31;
    MM++; // it might be February
  }

  if (((YY%4)==0)&&(MM==2)) {// it is a Leap Year
    if ((DD>29)&&(MM==2)){ // It is not February in a Leap Year
      DD = DD - 29;
      MM++; // it might be March
    }
  }
  else { // it is not a Leap Year
    if ((DD>28)&&(MM==2)){ // It is not February in a non Leap Year
      DD = DD - 28;
      MM++; // it might be March
    }
  }

  if ((DD>31)&&(MM==3)){ 
    DD = DD - 31; // It is not March
    MM++; // it might be April
  }

  if ((DD>30)&&(MM==4)){ 
    DD = DD - 30; // It is not April
    MM++; // it might be May
  }

  if ((DD>31)&&(MM==5)){ 
    DD = DD - 31; // It is not May
    MM++; // it might be June
  }

  if ((DD>30)&&(MM==6)){ 
    DD = DD - 30; // It is not June
    MM++; // it might be July
  }

  if ((DD>31)&&(MM==7)){ 
    DD = DD - 31; // It is not July
    MM++; // it might be Aug
  }

  if ((DD>31)&&(MM==8)){ 
    DD = DD - 31; // It is not Aug
    MM++; // it might be Sept
  }

  if ((DD>30)&&(MM==9)){ 
    DD = DD - 30; // It is not Sept
    MM++; // it might be Oct
  }

  if ((DD>31)&&(MM==10)){ 
    DD = DD - 31; // It is not Oct
    MM++; // it might be Nov
  }

  if ((DD>30)&&(MM==11)){ 
    DD = DD - 30; // It is not Nov
    MM++; // it must be December
  }

  Tstmp1.DD = decToBcd((byte)(DD));
  Tstmp1.MM = decToBcd((byte)(MM));
  Tstmp1.YY = decToBcd((byte)(YY));
}

void ConvertTSTMP2LV(){

  LV.Snumber = BCDTodec(Tstamp.ss) + BCDTodec(Tstamp.mm)*60 + BCDTodec(Tstamp.hh)*3600;
  LV.Dnumber = 36526; // 1-1-2000
  LV.Dnumber = LV.Dnumber + (BCDTodec(Tstamp.YY)/4)*1461; // There are 1461 days in 4 years, one of which is a leap day
  LV.Dnumber = LV.Dnumber + (BCDTodec(Tstamp.YY)%4)*365; // all remaining full years have 365 days
  if ((BCDTodec(Tstamp.YY)%4) == 0) LV.Dnumber--;
  
 switch (Tstamp.MM) {
    case (0x01):  // It's Januari
    LV.Dnumber = LV.Dnumber + BCDTodec(Tstamp.DD);
    break;

    case (0x02):  // It's Februari
    LV.Dnumber = LV.Dnumber + BCDTodec(Tstamp.DD) + 31 ;
    break;

    case (0x03):  // It's March
    LV.Dnumber = LV.Dnumber + BCDTodec(Tstamp.DD) + 31 + 28;
    if (BCDTodec(Tstamp.YY)%4 == 0) LV.Dnumber++;
    break;

    case (0x04):  // It's April
    LV.Dnumber = LV.Dnumber + BCDTodec(Tstamp.DD) + 31 + 28 + 31;
    if (BCDTodec(Tstamp.YY)%4 == 0) LV.Dnumber++;
    break;

    case (0x05):  // It's May
    LV.Dnumber = LV.Dnumber + BCDTodec(Tstamp.DD) + 31 + 28 + 31 + 30;
    if (BCDTodec(Tstamp.YY)%4 == 0) LV.Dnumber++;
    break;

    case (0x06):  // It's June
    LV.Dnumber = LV.Dnumber + BCDTodec(Tstamp.DD) + 31 + 28 + 31 + 30 + 31;
    if (BCDTodec(Tstamp.YY)%4 == 0) LV.Dnumber++;
    break;

    case (0x07):  // It's July
    LV.Dnumber = LV.Dnumber + BCDTodec(Tstamp.DD) + 31 + 28 + 31 + 30 + 31 + 30;
    if (BCDTodec(Tstamp.YY)%4 == 0) LV.Dnumber++;
    break;

    case (0x08):  // It's August
    LV.Dnumber = LV.Dnumber + BCDTodec(Tstamp.DD) + 31 + 28 + 31 + 30 + 31 + 30 + 31;
    if (BCDTodec(Tstamp.YY)%4 == 0) LV.Dnumber++;
    break;

    case (0x09):  // It's September
    LV.Dnumber = LV.Dnumber + BCDTodec(Tstamp.DD) + 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31;
    if (BCDTodec(Tstamp.YY)%4 == 0) LV.Dnumber++;
    break;

    case (0x10):  // It's October
    LV.Dnumber = LV.Dnumber + BCDTodec(Tstamp.DD) + 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30;
    if (BCDTodec(Tstamp.YY)%4 == 0) LV.Dnumber++;
    break;

    case (0x11):  // It's November
    LV.Dnumber = LV.Dnumber + BCDTodec(Tstamp.DD) + 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31;
    if (BCDTodec(Tstamp.YY)%4 == 0) LV.Dnumber++;
    break;

    case (0x12): // It's December
    LV.Dnumber = LV.Dnumber + BCDTodec(Tstamp.DD) + 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30;
    if (BCDTodec(Tstamp.YY)%4 == 0) LV.Dnumber++;
    break;
  } // end switch on months
} // End ConvertTSTMP2LV


void setup(){
  Serial.begin(115200);
  LVal_1.Dnumber  = 42000U;
  LVal_1.Snumber  = 0U;
  LVal_1.WL  = 0U;

}

void loop(){
  delay(150);
  LVal_1.Dnumber = LVal_1.Dnumber + 1;
  //  if (LVal_1.DN > 43909U) LVal_1.DN = LVal_1.DN - 1904U; // to keep dates in relevant time period
  LVal_1.Snumber = LVal_1.Snumber + 771;
  if (LVal_1.Snumber > 86399U) LVal_1.Snumber = LVal_1.Snumber - 86400U; // to keep seconds within the allowed range 0..86399

  Serial.print("Snumber: ");
  Serial.print(LVal_1.Snumber);
  Serial.print(", ");
  Serial.print("Dnumber: ");
  Serial.print(LVal_1.Dnumber);
  Serial.print(", ");

  ConvertLV2TSTMP();

  if (Tstmp1.hh < 0x10) Serial.print("0");
  Serial.print(Tstmp1.hh,HEX);
  Serial.print(":");
  if (Tstmp1.mm < 0x10) Serial.print("0");
  Serial.print(Tstmp1.mm,HEX);
  Serial.print(":");
  if (Tstmp1.ss < 0x10) Serial.print("0");
  Serial.print(Tstmp1.ss,HEX);
  Serial.print("  ");

  if (Tstmp1.DD < 10) Serial.print("0");
  Serial.print(Tstmp1.DD,HEX);
  Serial.print("-");
  if (Tstmp1.MM < 10) Serial.print("0");
  Serial.print(Tstmp1.MM,HEX);
  Serial.print("-");
  if (Tstmp1.YY < 10) Serial.print("0");
  Serial.print(Tstmp1.YY,HEX);
  Serial.print(", ");
  
  Tstamp = Tstmp1;
  
  ConvertTSTMP2LV();
  
  Serial.print("Snumber: ");
  Serial.print(LV.Snumber);
  Serial.print(", ");
  Serial.print("Dnumber: ");
  Serial.print(LV.Dnumber);
  Serial.println();
  

}

















