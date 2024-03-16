#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

 
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
 

 
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
 
double temp_amb;
double temp_obj;


void fillrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=3) {
    // The INVERSE color is used so rectangles alternate white/black
    if(i>20) {
      display.fillRect(i, i, display.width()-i*2, display.height()-i*2, BLACK);
    }
    else {
      display.fillRect(i, i, display.width()-i*2, display.height()-i*2, SSD1306_INVERSE);
    }
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(10);
}

void fillrect2(void) {

  for(int16_t i=21; i<display.height()/2; i+=3) {
    // The INVERSE color is used so rectangles alternate white/black
    display.fillRect(i, i, display.width()-i*2, display.height()-i*2, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(10);
}
 
void setup()
{
  pinMode(A3, INPUT); //set battery level as input
  
  
  Serial.begin(9600);
  mlx.begin();         //Initialize MLX90614
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //initialize with the I2C addr 0x3C (128x64)
 
  Serial.println("Temperature Sensor MLX90614");
  
  fillrect();

  display.setCursor(25,30);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("Dab Loading..");

  display.display();
  delay(3000);
  fillrect2();
}

int calcBatteryLvl() {
//     get battery % from input pin, Battery percentage has 
//      a voltage range of 4.2 @ 100% to 3.7 @ 10%.
//      Going lower than 3.7 is bad for the battery.
//      Use voltage divider to divide in half
  float batteryVolt = (analogRead(A3)/1023.0)*3.3;
  Serial.print("Battery Level = ");
  Serial.println(batteryVolt);
  return round((((batteryVolt*2)-3.5)/0.7)*100);
}

void loop()
{

  //Reading room temperature and object temp
  //for reading Fahrenheit values, use
  //mlx.readAmbientTempF() , mlx.readObjectTempF()
  //temp_amb = mlx.readAmbientTempF();
  int i;
  double sum=0;
  for (i=0; i<500; ++i) {
    sum += mlx.readObjectTempF();
  }
  temp_obj = sum/500.0;

  int batteryLvl =  calcBatteryLvl();

  //Serial Monitor
  Serial.print("Room Temp = ");
  Serial.println(temp_amb);
  Serial.print("Object temp = ");
  Serial.println(temp_obj);

  display.clearDisplay();
  display.fillScreen(WHITE);

  display.setCursor(5,5);  
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.print(batteryLvl);
  display.print("%");
  
  display.setCursor(25,10);  // 25,10
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.println(" Temperature");
  display.setCursor(25,30);
  display.setTextSize(2);
  display.print(temp_obj);
  display.print((char)247);
  display.print("F");
  display.display();
 
  delay(500);
}
