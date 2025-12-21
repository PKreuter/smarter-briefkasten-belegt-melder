
// OLED helper


/**

    display.clearDisplay() – all pixels are off
    display.drawPixel(x,y, color) – plot a pixel in the x,y coordinates
    display.setTextSize(n) – set the font size, supports sizes from 1 to 8
    display.setCursor(x,y) – set the coordinates to start writing text
    display.print(“message”) – print the characters at location x,y
    display.display() – call this method for the changes to make effect
**/


// Write row/line
void oled1WriteMsg(int x, int y, char msg[30]) {
  oled1.setTextColor(WHITE, BLACK);
  oled1.setTextSize(1);
  oled1.setCursor(x, y);
  oled1.print(msg);
  oled1.display();
}


// Clear row/line 
void oled1ClearRow(int x, int y) {
  oled1.setCursor(y, y);
  oled1.setTextColor(BLACK, BLACK);
  oled1.print("                                    ");
  oled1.setTextColor(WHITE, BLACK);
  oled1.display();
}

// Write row/line
void oled1ClearWriteMsg(int x, int y, char msg[30]) {
  oled1ClearRow(x, y);
  oled1WriteMsg(x, y, msg);
}



void fillScreen(int Color) {

}




