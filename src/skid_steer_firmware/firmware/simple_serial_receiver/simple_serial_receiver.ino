void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(Serial.available()){
    #ifdef RGB_BUILTIN
    int x = Serial.readString().toInt();
    if(x == 0){
      digitalWrite(RGB_BUILTIN, LOW);
    }else{
      rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, RGB_BRIGHTNESS);
    }
    #endif
  }
  
}
