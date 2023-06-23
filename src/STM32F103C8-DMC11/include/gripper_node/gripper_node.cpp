#include <gripper_node.hpp>

using namespace GripperManagerNode;


GripperManager::GripperManager()
{

}

GripperManager::~GripperManager()
{

}

uint8_t EncoderInit()
{
    Serial.println("Encoder interrupter Initializing...");
    pinMode(ENCODER_PHASE_A, INPUT);
    pinMode(ENCODER_PAHSE_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PHASE_A), isr_encoder_phase_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PAHSE_B), isr_encoder_phase_B, CHANGE);

    g_enc_pos = 0;
    g_enc_pos = EEPROMReadlong(ENCODER_POS_EEPROM_ADDRESS);
    Serial.print("DONE");
    Serial.print(" / EEPROM_pos : ");
    Serial.print(g_enc_pos);
    Serial.print(" / target Pos : ");
    Serial.println(TARGET_POS);

    return 1;
}

uint8_t MotorInit()
{
    Serial.print("Motor Pin & PWM Initializing...");

    pinMode(MOTOR_CW, OUTPUT);
    pinMode(MOTOR_CCW, OUTPUT);

    digitalWrite(MOTOR_CW, HIGH);
    digitalWrite(MOTOR_CCW, LOW);
    // put your setup code here, to run once:

    pinMode(MOTOR_PWM_PIN, PWM);

    HardwareTimer pwmtimer2(2);
    pwmtimer2.pause();
    pwmtimer2.setCount(0);
    pwmtimer2.setPrescaleFactor(1);  // 72/1 = 72
    pwmtimer2.setOverflow(3600 - 1); // 72mHz/3600 = 20 (kHz)
    pwmtimer2.refresh();
    pwmtimer2.resume();

    pwmWrite(MOTOR_PWM_PIN, DUTY_MIN);

    Serial.println("DONE");

    return 1;
}

uint8_t LoadCellInit()
{
    Serial.print("Load Cell Inititializing...");

    LoadCell_1.begin();
    LoadCell_2.begin();
    LoadCell_3.begin();

    unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
    boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
    byte loadcell_1_rdy = 0;
    byte loadcell_2_rdy = 0;
    byte loadcell_3_rdy = 0;

    while ((loadcell_1_rdy + loadcell_2_rdy) < 2)
    { // run startup, stabilization and tare, both modules simultaniously
        if (!loadcell_1_rdy)
            loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
        if (!loadcell_2_rdy)
            loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
        if (!loadcell_3_rdy)
            loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
    }

    if (LoadCell_1.getTareTimeoutFlag())
    {
        Serial.print("\n");
        Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
    }

    if (LoadCell_2.getTareTimeoutFlag())
    {
        Serial.print("\n");
        Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
    }

    if (LoadCell_3.getTareTimeoutFlag())
    {
        Serial.print("\n");
        Serial.println("Timeout, check MCU>HX711 no.3 wiring and pin designations");
    }

    LoadCell_1.setCalFactor(CALIBRATION_VALUE); // user set calibration value (float)
    LoadCell_2.setCalFactor(CALIBRATION_VALUE); // user set calibration value (float)
    LoadCell_3.setCalFactor(CALIBRATION_VALUE); // user set calibration value (float)
    Serial.println("Done");

    return 1;
}

uint8_t RTOSInit()
{
    Serial.println("RTOS Thread Creating...");

    xTaskCreate(MotorOperation,
                "Task1",
                256,
                NULL,
                1,
                NULL);

    xTaskCreate(LoadCellUpdate,
                "Task2",
                1024,
                NULL,
                2,
                NULL);

    xTaskCreate(EncoderMonitor,
                "Task3",
                128,
                NULL,
                0,
                NULL);

    xTaskCreate(EEPROMSave,
                "Task4",
                200,
                NULL,
                tskIDLE_PRIORITY,
                NULL);

    xTaskCreate(task1,
                "Task5",
                64,
                NULL,
                tskIDLE_PRIORITY,
                NULL);

    vTaskStartScheduler();
    Serial.println("RTOS Thread Created!");

    return 1;
}

void isr_encoder_phase_A()
{
    if (digitalRead(ENCODER_PHASE_A) == HIGH)
    {
        // check channel B to see which way encoder is turning
        if (digitalRead(ENCODER_PAHSE_B) == LOW)
        {
            g_enc_pos += 1;
        }
        else
        {
            g_enc_pos -= 1;
        }
    }
    else // must be a high-to-low edge on channel A
    {
        // check channel B to see which way encoder is turning
        if (digitalRead(ENCODER_PAHSE_B) == HIGH)
        {
            g_enc_pos += 1;
        }
        else
        {
            g_enc_pos -= 1;
        }
    }
}

void isr_encoder_phase_B()
{
    // look for a low-to-high on channel B
    if (digitalRead(ENCODER_PAHSE_B) == HIGH)
    {
        // check channel A to see which way encoder is turning
        if (digitalRead(ENCODER_PHASE_A) == HIGH)
        {
            g_enc_pos += 1;
        }
        else
        {
            g_enc_pos -= 1;
        }
    }
    // Look for a high-to-low on channel B
    else
    {
        // check channel B to see which way encoder is turning
        if (digitalRead(ENCODER_PHASE_A) == LOW)
        {
            g_enc_pos += 1;
        }
        else
        {
            g_enc_pos -= 1;
        }
    }
}

void LoadCellUpdate()
{

}
