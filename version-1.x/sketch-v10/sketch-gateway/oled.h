
// OLED helper

// Write row/line
void oledWriteMsg(int x, int y, char msg[30])
{
  display.setTextColor(WHITE, BLACK);
  display.setTextSize(1);
  display.setCursor(x, y);
  display.print(msg);
  display.display();
}


// Clear row/line 
void oledClearRow(int x, int y) {
  display.setCursor(y, y);
  display.setTextColor(BLACK, BLACK);
  display.print("                                    ");
  display.setTextColor(WHITE, BLACK);
  display.display();
}

// Write row/line
void oledClearWriteMsg(int x, int y, char msg[30]) {
  oledClearRow(x, y);
  oledWriteMsg(x, y, msg);
}




