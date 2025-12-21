



// OLED helper

//--
// Clear row/line 
void oledClearRow(int row) {
  display.setCursor(0, row);
  display.setTextColor(BLACK, BLACK);
  display.print("                                    ");
  display.setTextColor(WHITE, BLACK);
  display.display();
}



// Write row/line
void oledWriteMsg(int row, char msg[30]) {
  display.setTextColor(WHITE, BLACK);
  display.setTextSize(1);
  display.setCursor(0, row);
  display.print(msg);
  display.display();
}

// Write row/line
void oledClearWriteMsg(int row, char msg[30]) {
  oledClearRow(row);
  oledWriteMsg(row, msg);
}
//---