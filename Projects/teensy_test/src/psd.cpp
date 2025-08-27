#include <Arduino.h>

void setup(){
    Serial.begin(57600);
}

void psd(int psd_pin){
    float data_A = 0;
    float data_A1 ;
    float data_A2 ;

    for(int i = 0; i < 100; i++){
        data_A = data_A + analogRead(psd_pin);
    }

    data_A = data_A/100;
    data_A1 = 3.3 * data_A /1024;
    data_A2 = 27.149*pow(data_A1,-1.185);  
    //Serial.print(analogRead(22));
    Serial.print(data_A1);
    Serial.print("   "); 
    Serial.print(data_A2); 
    Serial.println(" cm");
    delay(100);

    return;
}

void loop(){
    psd(22);
}