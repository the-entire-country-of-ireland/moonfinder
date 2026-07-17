// #include "LGFX_ESP32_2432S028R_CYD.hpp"

// static LGFX lcd;
// static LGFX_Sprite sprite(&lcd);

// // If you are using TFT_eSPI and would like to avoid changing the source code so much, you can use this header.
// // #include <LGFX_TFT_eSPI.hpp>
// // static TFT_eSPI lcd;               // TFT_eSPI is defined as an alias for LGFX.
// // static TFT_eSprite sprite(&lcd);   // TFT_eSprite is defined as an alias for LGFX_Sprite.

// void setup(void) {
//   Serial.begin(115200);
//   while (!Serial || millis() < 1000);

//   lcd.init();
//   lcd.initDMA();
//   lcd.setRotation(2);
//   lcd.setBrightness(128);  // 0 ~ 255

//   // Set the color mode as needed (default is 16).
//   // 16 requires less SPI communication and operates faster, but the red and blue gradations are 5 bits.
//   // 24 requires more SPI communication, but produces clearer tonal expression.
//   lcd.setColorDepth(16);  // Set to 16-bit RGB565
// //lcd.setColorDepth(24);  // Set to 24-bit RGB888 (the number of colors displayed will be 18-bit RGB666 depending on the panel)

//   if (!lcd.touch()) {
//     Serial.println("Touch device not found.");
//   }

//   // The basic drawing functions are:
//   /*
//   fillScreen    (                color);  // Fill the entire screen
//   drawPixel     ( x, y         , color);  // Draw a point
//   drawFastVLine ( x, y   , h   , color);  // Vertical  Line
//   drawFastHLine ( x, y, w      , color);  // Horizonal Line
//   drawRect      ( x, y, w, h   , color);  // Rectangle perimeter
//   fillRect      ( x, y, w, h   , color);  // Rectangle fill
//   drawRoundRect ( x, y, w, h, r, color);  // Rounded rectangle with perimeter 
//   fillRoundRect ( x, y, w, h, r, color);  // Rounded rectangle with filling
//   drawCircle    ( x, y      , r, color);  // Circle  with circumference
//   fillCircle    ( x, y      , r, color);  // Circle  with filling
//   drawEllipse   ( x, y, rx, ry , color);  // Ellipse with circumference
//   fillEllipse   ( x, y, rx, ry , color);  // Ellipse with filling
//   drawLine      ( x0, y0, x1, y1        , color); // Straight line between two points
//   drawTriangle  ( x0, y0, x1, y1, x2, y2, color); // Perimeter of a triangle between three points
//   fillTriangle  ( x0, y0, x1, y1, x2, y2, color); // Filling a triangle between three points
//   drawBezier    ( x0, y0, x1, y1, x2, y2, color); // 3-points Bezier curve
//   drawBezier    ( x0, y0, x1, y1, x2, y2, x3, y3, color); // 4-points Bezier curve
//   drawArc       ( x, y, r0, r1, angle0, angle1, color);   // Arc perimeter
//   fillArc       ( x, y, r0, r1, angle0, angle1, color);   // Arc filling
// */


//   // Drawing a point by drawPixel() with 3 arguments: X coordinate, Y coordinate, and color.
//   lcd.drawPixel(0, 0, 0xFFFF);  // Draw a white dot at coordinate 0:0


//   // There is a function to generate color codes that can be used to specify colors.
//   // The arguments specify red, green, and blue as values ​​between 0 and 255.
//   // Using color888 is recommended to avoid loss of color information.
//   lcd.drawFastVLine(2, 0, 100, lcd.color888(255, 0, 0));  // Draw a vertical line in red
//   lcd.drawFastVLine(4, 0, 100, lcd.color565(0, 255, 0));  // Draw a vertical line in green
//   lcd.drawFastVLine(6, 0, 100, lcd.color332(0, 0, 255));  // Draw a vertical line in blue


//   // You can also use the following type
//   // RGB888 24 bits (type: uint32_t)
//   // RGB565 16 bits (type: uint16_t, uint32_t)
//   // RGB332  8-bits (type: uint8_t)

//   // If you use the uint32_t type, it will be treated as 24-bit RGB888.
//   // It can be written in two hexadecimal digits in the order of red, green and blue.
//   // Use a variable of type uint32_t, add a "U" to the end, or cast it to type uint32_t.
//   uint32_t red = 0xFF0000;
//   lcd.drawFastHLine(0, 2, 100, red);             // Draw a vertical line in red
//   lcd.drawFastHLine(0, 4, 100, 0x00FF00U);       // Draw a vertical line in green
//   lcd.drawFastHLine(0, 6, 100, (uint32_t)0xFF);  // Draw a vertical line in blue


//   // If you use the uint16_t or int32_t type, it will be treated as 16-bit RGB565.
//   // If no special writing is done, it will be treated as an int32_t type.
//   // (This is for compatibility with AdafruitGFX and TFT_eSPI.)
//   uint16_t green = 0x07E0;
//   lcd.drawRect(10, 10, 50, 50, 0xF800);          // Draw the outline of the rectangle in red
//   lcd.drawRect(12, 12, 50, 50, green);           // Draw the outline of the rectangle in green
//   lcd.drawRect(14, 14, 50, 50, (uint16_t)0x1F);  // Draw the outline of the rectangle in blue


//   // If you use the int8_t or uint8_t type, it will be treated as 8-bit RGB332.
//   uint8_t blue = 0x03;
//   lcd.fillRect(20, 20, 20, 20, (uint8_t)0xE0);  // Draw a rectangle filled in red
//   lcd.fillRect(30, 30, 20, 20, (uint8_t)0x1C);  // Draw a rectangle filled in green
//   lcd.fillRect(40, 40, 20, 20, blue);           // Draw a rectangle filled in blue


//   // The color argument to the drawing function is optional.
//   // If omitted, the color set by the setColor() function or the last used color will be applied.
//   // If you are drawing repeatedly with the same color, omitting it will run slightly faster.
//   lcd.setColor(0xFF0000U);                         // Specify red as the drawing color
//   lcd.fillCircle(40, 80, 20);                      // Fill a circle with red
//   lcd.fillEllipse(80, 40, 10, 20);                 // Fill an oval  with red
//   lcd.fillArc(80, 80, 20, 10, 0, 90);              // Fill an arc   with red
//   lcd.fillTriangle(80, 80, 60, 80, 80, 60);        // Fill a triangular with red

//   lcd.setColor(0x0000FFU);                         // Specify blue as the drawing color
//   lcd.drawCircle(40, 80, 20);                      // Blue circle perimeter
//   lcd.drawEllipse(80, 40, 10, 20);                 // Blue oval outline
//   lcd.drawArc(80, 80, 20, 10, 0, 90);              // Blue arc circumference
//   lcd.drawTriangle(60, 80, 80, 80, 80, 60);        // Blue triangular perimeter

//   lcd.setColor(0x00FF00U);                         // Specify green as the drawing color
//   lcd.drawBezier(60, 80, 80, 80, 80, 60);          // Green quadratic Bezier curve
//   lcd.drawBezier(60, 80, 80, 20, 20, 80, 80, 60);  // Green cubic Bezier curve

//   // When using drawGradientLine(), which draws a gradient line, you cannot omit specifying the color.
//   lcd.drawGradientLine(0, 80, 80, 0, 0xFF0000U, 0x0000FFU);  // Red to blue gradient line

//   delay(1000);

//   // You can fill the entire screen with clear() or fillScreen().
//   // fillScreen is the same as specifying the entire screen with fillRect,
//   // and the color specified will be treated as the drawing color.
//   lcd.fillScreen(0xFFFFFFu);  // Fill with White
//   lcd.setColor(0x00FF00u);    // Specify green as the drawing color
//   lcd.fillScreen();           // Fill with green

//   // clear() is separate from drawing functions and stores the color as the background color.
//   // The background color is not used often, but it is also used to fill in the gaps when using the scrolling function.
//   lcd.clear(0xFFFFFFu);         // Fill the background with white
//   lcd.setBaseColor(0x000000u);  // Specify black as the background color
//   lcd.clear();                  // Fill with background color


//   // The SPI bus is automatically allocated and released when a drawing function is called,
//   // but if you need speed, use startWrite() and endWrite() before and after the drawing process.
//   // The acquisition and release of the SPI bus is suppressed, improving speed.
//   // In the case of electronic paper display (EPD), the drawings made after startWrite()
//   // are reflected on the screen by calling endWrite().
//   lcd.drawLine(0, 1, 39, 40, red);        // Acquire SPI bus, draw line, release SPI bus
//   lcd.drawLine(1, 0, 40, 39, blue);       // Acquire SPI bus, draw line, release SPI bus

//   lcd.startWrite();                       // Acquire SPI bus
//   lcd.drawLine(38, 0, 0, 38, 0xFFFF00U);  // Draw a line
//   lcd.drawLine(39, 1, 1, 39, 0xFF00FFU);  // Draw a line
//   lcd.drawLine(40, 2, 2, 40, 0x00FFFFU);  // Draw a line
//   lcd.endWrite();                         // Release SPI bus


//   // startWrite() and endWrite() internally count the number of times they are called, 
//   // and if they are called repeatedly, only the first and last will work.
//   // Always use startWrite() and endWrite() as a pair.
//   // (If you never mind occupying the SPI bus, you can call startWrite() once at the beginning without calling endWrite().)
//   lcd.startWrite();  // Count +1, acquire SPI bus
//   lcd.startWrite();  // Count +1
//   lcd.startWrite();  // Count +1
//   lcd.endWrite();    // Count -1
//   lcd.endWrite();    // Count -1
//   lcd.endWrite();    // Count -1, release SPI bus
//   lcd.endWrite();    // Do nothing (Excessive calls to endWrite() have no effect and the count never goes negative.)


//   // If you want to forcibly release and acquire the SPI bus regardless of the
//   // startWrite() count status, use endTransaction() and beginTransaction().
//   // The count is not cleared, so be careful not to create inconsistencies.
//   lcd.startWrite();      // Count +1, acquire SPI bus
//   lcd.startWrite();      // Count +1
//   lcd.drawPixel(0, 0);   // Drawing
//   lcd.endTransaction();  // Release SPI bus
//   // Other SPI devices can be used here
//   // When using another device (such as an SD card) on the same SPI bus,
//   // make sure the SPI bus is free.
//   lcd.beginTransaction();  // Acquire SPI bus
//   lcd.drawPixel(0, 0);     // Drawing
//   lcd.endWrite();          // Count -1
//   lcd.endWrite();          // Count -1, release SPI bus


//   // In addition to drawPixel(), there is a function called writePixel() that draws points.
//   // While drawPixel allocates the SPI bus if necessary, writePixel does not check the state of the SPI bus.
//   lcd.startWrite();  // acquire SPI bus
//   for (uint32_t x = 0; x < 128; ++x) {
//     for (uint32_t y = 0; y < 128; ++y) {
//       lcd.writePixel(x, y, lcd.color888(x * 2, x + y, y * 2));
//     }
//   }
//   lcd.endWrite();  // Aelease SPI bus
//                    // All functions whose names begin with write~ must have an explicit call to startWrite.
//                    // Ex) writePixel()、writeFastVLine()、writeFastHLine()、writeFillRect()

//   delay(1000);

//   // The same drawing functions can also be used to draw to sprites (offscreen).
//   // First, specify the color depth of the sprite with setColorDepth(). (If omitted, it will be treated as 16.)
//   //sprite.setColorDepth(1);   // Set to 1-bit (2-colors)  palette mode
//   //sprite.setColorDepth(2);   // Set to 2-bit (4-colors)  palette mode
//   //sprite.setColorDepth(4);   // Set to 4-bit (16-colors) palette mode
//   //sprite.setColorDepth(8);   // Set to 8-bit RGB332
//   //sprite.setColorDepth(16);  // Set to 16-bit RGB565
//   sprite.setColorDepth(24);  // Set to 24-bit RGB888


//   // By setting setColorDepth(8) and then calling createPalette(), the 256-colors palette mode will be activated.
//   // sprite.createPalette();


//   // Specify width and height in createSprite() to allocate memory.
//   // The memory consumed is proportional to the color depth and area.
//   // Please note that if it is too large, memory allocation will fail.
//   sprite.createSprite(65, 65);  // Create a sprite with width 65 and height 65.

//   for (uint32_t x = 0; x < 64; ++x) {
//     for (uint32_t y = 0; y < 64; ++y) {
//       sprite.drawPixel(x, y, lcd.color888(3 + x * 4, (x + y) * 2, 3 + y * 4));  // Draw on sprite
//     }
//   }
//   sprite.drawRect(0, 0, 65, 65, 0xFFFF);

//   // The sprite you create can be output to any coordinate using pushSprite().
//   // The output destination will be the LGFX passed as an argument when creating the instance.
//   sprite.pushSprite(64, 0);  // Draw a sprite at coordinates (64, 0) on the lcd

//   // If you did not pass a pointer to the drawing destination when creating a sprite instance,
//   // or if there are multiple LGFX, you can also use pushSprite() by specifying the output destination
//   // as the first argument.
//   sprite.pushSprite(&lcd, 0, 64);  // Draw a sprite at coordinates (0, 64) on the lcd

//   delay(1000);

//   // You can rotate, zoom, and draw sprites with pushRotateZoom().
//   // The coordinates set by setPivot() are treated as the center of rotation,
//   // and the destination coordinates are drawn so that the center of rotation is located there.
//   sprite.setPivot(32, 32);  // Coordinates (32, 32) are treated as the center
//   int32_t center_x = lcd.width() / 2;
//   int32_t center_y = lcd.height() / 2;
//   lcd.startWrite();
//   for (int angle = 0; angle <= 360; ++angle) {
//     // Draw at the center of the screen at an angle of 2.5 times the width and 3 times the height
//     sprite.pushRotateZoom(center_x, center_y, angle, 2.5, 3);

//     // In the case of electronic paper, the display is updated once every 36 times.
//     if ((angle % 36) == 0) lcd.display();
//   }
//   lcd.endWrite();

//   delay(1000);

//   // Use deleteSprite() to free the memory of a sprite that you are no longer using.
//   sprite.deleteSprite();

//   // Even after deleteSprite(), the same instance can be reused.
//   sprite.setColorDepth(4);      // Set to 4-bit (16-colors) palette mode
//   sprite.createSprite(65, 65);  // Create another sprite in the same instance

//   // In palette mode sprites, the color arguments to drawing functions are treated as palette numbers.
//   // When drawing with pushSprite() etc., the actual drawing color is determined by referring to the palette.

//   // In 4-bit (16-color) palette mode, palette numbers 0 to 15 can be used.
//   // The initial color of the palette is 0, black, and the final color of the palette is white,
//   // with a gradient from 0 to the end.
//   // To set the palette color, use setPaletteColor().
//   sprite.setPaletteColor(1, 0x0000FFU);  // Set palette number 1 to blue
//   sprite.setPaletteColor(2, 0x00FF00U);  // Set palette number 2 to green
//   sprite.setPaletteColor(3, 0xFF0000U);  // Set palette number 3 to red

//   sprite.fillRect(10, 10, 45, 45, 1);              // Paint a rectangle with palette number 1
//   sprite.fillCircle(32, 32, 22, 2);                // Paint a circle    with palette number 2
//   sprite.fillTriangle(32, 12, 15, 43, 49, 43, 3);  // Paint a triangles with palette number 3

//   // The last argument to pushSprite() allows you to specify a color that you do not want to draw.
//   sprite.pushSprite(0, 0, 0);    // Draw sprite with palette 0 as transparent
//   sprite.pushSprite(65, 0, 1);   // Draw sprite with palette 1 as transparent
//   sprite.pushSprite(0, 65, 2);   // Draw sprite with palette 2 as transparent
//   sprite.pushSprite(65, 65, 3);  // Draw sprite with palette 3 as transparent

//   delay(5000);

//   lcd.startWrite();  // By calling startWrite() here, the SPI bus will remain occupied.
// }

// void touchTest() {
//   uint16_t x, y;
//   if (lcd.getTouch(&x, &y)) {
//     Serial.println("x: " + String(x) + ", y: " + String(y));

//     // Draw a white spot at the detected coordinates
//     lcd.fillCircle(x, y, 2, TFT_WHITE);
//   }
// }

// void loop(void) {
//   static int count = 0;
//   static int a = 0;
//   static int x = 0;
//   static int y = 0;
//   static float zoom = 3;
//   ++count;
//   if ((a += 1) >= 360) a -= 360;
//   if ((x += 2) >= lcd.width()) x -= lcd.width();
//   if ((y += 1) >= lcd.height()) y -= lcd.height();
//   sprite.setPaletteColor(1, lcd.color888(0, 0, count & 0xFF));
//   sprite.setPaletteColor(2, lcd.color888(0, ~count & 0xFF, 0));
//   sprite.setPaletteColor(3, lcd.color888(count & 0xFF, 0, 0));

//   sprite.pushRotateZoom(x, y, a, zoom, zoom, 0);

//   touchTest();

//   // In the case of electronic paper, the display is updated once every 100 times.
//   if ((count % 100) == 0) lcd.display();
// }