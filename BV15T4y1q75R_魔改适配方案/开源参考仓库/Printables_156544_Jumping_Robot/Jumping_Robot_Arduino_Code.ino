int B1A = 2;
int B1B = 3;
int A1A = 5;
int A1B = 4;

#define CH1 11
#define CH2 10
#define CH3 9
#define CH6 6

int ch1Value;
int ch2Value;
int ch3Value;

bool ch6Value;


int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}


bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void setup(){
  
 pinMode (B1A, OUTPUT);
 pinMode (B1B, OUTPUT);
 pinMode (A1A, OUTPUT);
 pinMode (A1B, OUTPUT);
 pinMode (13,OUTPUT);
  
  
  
  Serial.begin(115200);
  
  
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH6, INPUT);
}


void loop() {
  
  
  ch1Value = readChannel(CH1, 0, 100, 50);
  ch2Value = readChannel(CH2, -100, 100, 0);
  ch3Value = readChannel(CH3, -100, 100, -100);
  ch6Value = readSwitch(CH6, false);
  
 
  Serial.print("Ch1: ");
  Serial.print(ch1Value);
  Serial.print(" | Ch2: ");
  Serial.print(ch2Value);
  Serial.print(" | Ch3: ");
  Serial.print(ch3Value);
  Serial.print(" | Ch6: ");
  Serial.println(ch6Value);
  
 
 if (ch2Value > -50 && ch2Value < 50 ) 
  
  {
   digitalWrite (B1A, LOW); 
   digitalWrite (B1B, LOW); 
   digitalWrite (A1A, LOW);  
   digitalWrite (A1B, LOW); 
  }

if (ch2Value < -80) 
  {
   digitalWrite (B1A, LOW); 
   digitalWrite (B1B, HIGH); 
   digitalWrite (A1A, HIGH);  
   digitalWrite (A1B, LOW); 
  }

if (ch2Value > 50) 
  {
   digitalWrite (B1A, HIGH); 
   digitalWrite (B1B, LOW); 
   digitalWrite (A1A, LOW);  
   digitalWrite (A1B, HIGH); 
  }



 
 
 

if (ch1Value > 70) 
  {
   analogWrite (B1A, HIGH); 
   digitalWrite (B1B, LOW); 
   analogWrite (A1A, HIGH);  
   digitalWrite (A1B, LOW); 
  }

if (ch1Value < 20) 
  
  {
   digitalWrite (B1A, LOW); 
   digitalWrite (B1B, HIGH); 
   digitalWrite (A1A, LOW);  
   digitalWrite (A1B, HIGH); 
  }



if (ch3Value > -50 && ch2Value < 50 ) 
  
  {
  digitalWrite (13, LOW);
  }

if (ch3Value > 50) 
  
  {
  digitalWrite (13, HIGH); 
  }

if (ch3Value < -50) 
 {
  digitalWrite (13, LOW);
   digitalWrite (B1A, HIGH); 
   digitalWrite (B1B, LOW); 
   digitalWrite (A1A, HIGH);  
   digitalWrite (A1B, LOW);
 
 
 }


}
