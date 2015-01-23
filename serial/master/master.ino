
void setup()
{
  Serial.begin(9600);
  Serial.setintr(test);
}

void loop()
{ 
  Serial.println("please type some strings.");
  delay(3000);
 
}
