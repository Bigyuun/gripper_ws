// Encoder pin (interrupt)
#define ENCODER_PHASE_A  PA0    
#define ENCODER_PAHSE_B  PA1

static long g_enc_pos = 0;


void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  Serial.println("Encoder interrupter Initializing...");
  pinMode(ENCODER_PHASE_A, INPUT);
  pinMode(ENCODER_PAHSE_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PHASE_A), isr_encoder_phase_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PAHSE_B), isr_encoder_phase_B, CHANGE);

  g_enc_pos = 0;
  // g_enc_pos = EEPROMReadlong(ENCODER_POS_EEPROM_ADDRESS);
  Serial.print("DONE");
  Serial.print(" / EEPROM_pos : "); Serial.print(g_enc_pos);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(g_enc_pos);
}




void isr_encoder_phase_A()
{
  if (digitalRead(ENCODER_PHASE_A) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_PAHSE_B) == LOW) {  
      g_enc_pos += 1;
    } 
    else {
      g_enc_pos -= 1;
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(ENCODER_PAHSE_B) == HIGH) {   
      g_enc_pos += 1;
    } 
    else {
      g_enc_pos -= 1;
    }
  }
}

void isr_encoder_phase_B()
{
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_PAHSE_B) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER_PHASE_A) == HIGH) {  
      g_enc_pos += 1;
    } 
    else {
      g_enc_pos -= 1;
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(ENCODER_PHASE_A) == LOW) {   
      g_enc_pos += 1;
    } 
    else {
      g_enc_pos -= 1;
    }
  }
}
