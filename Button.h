void buttonPressed()
{
    if (interrupt_process_status == !BUTTON_TRIGGERED) 
    {
      if (digitalRead(BUTTON_INPUT) == HIGH) 
      {
        interrupt_process_status  = BUTTON_TRIGGERED;
      }
    }
}

void initButton()
{
  pinMode(BUTTON_INPUT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_INPUT), buttonPressed, RISING);
}

bool readButton()
{
  int button_reading;
  
  static bool switching_pending = false;
  static  long int elapse_timer;
  if (interrupt_process_status == BUTTON_TRIGGERED) 
  {    
    button_reading = digitalRead(BUTTON_INPUT);
    if (button_reading == HIGH)  
    {
      switching_pending = true;
      elapse_timer  = millis();
    }
    if (switching_pending  && button_reading == LOW) 
    {
      if (millis() - elapse_timer >= DEBOUNCE) 
      {
        switching_pending  = false;
        interrupt_process_status  = !BUTTON_TRIGGERED;
        return BUTTON_SWITCHED;
      }
    }
  }
  
  return !BUTTON_SWITCHED;
}