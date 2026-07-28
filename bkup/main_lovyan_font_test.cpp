// #include <LovyanGFX.h>
// #define DISPLAY_CYD_2USB  true
// #include "LGFX_ESP32_2432S028R_CYD.hpp"

// static LGFX lcd;

// void drawGradation(void) {
//   // Draw a gradient in the background
//   lcd.startWrite();
//   lcd.setAddrWindow(0, 0, lcd.width(), lcd.height());
//   for (int y = 0; y < lcd.height(); ++y) {
//     for (int x = 0; x < lcd.width(); ++x) {
//       lcd.writeColor(lcd.color888(x >> 1, (x + y) >> 2, y >> 1), 1);
//     }
//   }
//   lcd.endWrite();
// }

// void setup(void) {
//   Serial.begin(115200);
//   while (!Serial || millis() < 1000);

//   lcd.init();
//   lcd.setRotation(2);

//   drawGradation();

//   // There are two main ways to draw characters: print() and drawString().

//   // The first argument of the drawString() is the string,
//   // the second argument is the X coordinate,
//   // and the third argument is the Y coordinate.
//   lcd.drawString("string!", 10, 10);

//   // The first argument to the drawNumber() function is the number.
//   lcd.drawNumber(123, 100, 10);

//   // The first argument of the drawFloat() is the number,
//   // the second argument is the number of decimal places,
//   // the third argument is the X coordinate,
//   // and the fourth argument is the Y coordinate.
//   lcd.drawFloat(3.14, 2, 150, 10);

//   // The print() draws at the coordinates specified by the setCursor function
//   // (or the continuation of the last character drawn by the print function).
//   lcd.setCursor(10, 20);
//   lcd.print("print!");

//   // The printf() can be used to draw the contents of the second and subsequent arguments.
//   // (It is compliant with the C language printf(), so it can also draw strings and floating-point numbers.)
//   int value = 123;
//   lcd.printf("test %d", value);

//   // The println() allows you to insert a newline after drawing a string.
//   // This has the same effect as print("\n");.
//   lcd.println("println");

//   // To change the font, use the setFont().
//   // The fonts that are the same as the setTextFont() of TFT_eSPI are Font0 to Font8.
//   // If editor's input assistance is available, you can find font list by entering "&fonts::" in the argument.
//   lcd.setFont(&fonts::Font4);
//   lcd.println("TestFont4");

//   // For compatibility with TFT_eSPI, it also supports changing the font by number using the setTextFont().
//   // The numbers that can be specified as arguments are 0, 2, 4, 6, 7, and 8. (TFT_eSPI compliant)
//   // This method is not recommended because the binary will include other numeric fonts, which makes it inflating.
//   lcd.setTextFont(2);
//   lcd.println("TestFont2");


//   // You can change the color with setTextColor().
//   // The first argument is the text color, the second argument is the background color.
//   lcd.setTextColor(0x00FFFFU, 0xFF0000U);
//   lcd.print("CyanText RedBack");
//   // If you want to repeatedly redraw text in the same place, specifying a background color is recommended to overwrite it.
//   // If you erase using fillRect() etc. and then rewrite it, flickering may occur.


//   // If you specify only the first argument to setTextColor and omit the second,
//   // the background will not be filled and only the text will be drawn.
//   lcd.setTextColor(0xFFFF00U);
//   lcd.print("YellowText ClearBack");


//   // Font6 contains only characters for clocks.
//   lcd.setFont(&fonts::Font6);
//   lcd.print("apm.:-0369");

//   // Font7 contains fonts that look like 7-segment LCD displays.
//   lcd.setFont(&fonts::Font7);
//   lcd.print(".:-147");

//   // Font8 contains only numbers.
//   lcd.setFont(&fonts::Font8);
//   lcd.print(".:-258");


//   delay(3000);
//   drawGradation();

//   // There are 36 preset Japanese fonts converted from IPA fonts (4 types x 9 sizes).
//   // Trailing digits indicates the size, and available sizes are 8, 12, 16, 20, 24, 28, 32, 36, and 40.
//   // fonts::lgfxJapanMincho_12      // Mincho size 12 fixed width font
//   // fonts::lgfxJapanMinchoP_16     // Mincho size 16 proportional font
//   // fonts::lgfxJapanGothic_20      // Gothic size 20 fixed width font
//   // fonts::lgfxJapanGothicP_24     // Gothic size 24 proportional font

//   // There are 20 preset fonts (4 types x 5 sizes each) for Japanese, Korean, and Chinese
//   // (Simplified and Traditional) converted from efonts.
//   // Trailing digits represent the size and are available in 10, 12, 14, 16 and 24.
//   // The letter at the end stands for b=bold / i=italic.
//   // fonts::efontJA_10              // Japanese size 10
//   // fonts::efontCN_12_b            // Simplified Chinese Size 12 Bold
//   // fonts::efontTW_14_bi           // Traditional Size 14 Bold Italic
//   // fonts::efontKR_16_i            // Korean Size 16 Italic

//   lcd.setCursor(0, 0);
//   lcd.setFont(&fonts::lgfxJapanMincho_16);
//   lcd.print("明朝体 16 Hello World\nこんにちは世界\n");
//   //lcd.setFont(&fonts::lgfxJapanMinchoP_16);  lcd.print("明朝 P 16 Hello World\nこんにちは世界\n");
//   lcd.setFont(&fonts::lgfxJapanGothic_16);
//   lcd.print("ゴシック体 16 Hello World\nこんにちは世界\n");
//   //lcd.setFont(&fonts::lgfxJapanGothicP_16);  lcd.print("ゴシック P 16 Hello World\nこんにちは世界\n");

//   // By using Yamaneko's [Japanese Font Subset Generator](https://github.com/yamamaya/lgfxFontSubsetGenerator),
//   // you can create small-sized font data that contains only the characters you need.


//   delay(3000);
//   drawGradation();


//   // LovyanGFX also supports AdafruitGFX fonts using the setFont() function.
//   // (The setFreeFon()t function is also provided for compatibility with TFT_eSPI.)
//   lcd.setFont(&fonts::FreeSerif9pt7b);


//   // If you want to draw right-justified or centered, specify the reference position using the setTextDatum function.
//   // There are four vertical orientations: top, middle, baseline, and bottom.
//   // And three horizontal orientations: left, center, and right.
//   // You can choose from 12 different combinations of vertical and horizontal orientations.
//   lcd.setTextDatum(textdatum_t::top_left);
//   lcd.setTextDatum(textdatum_t::top_center);
//   lcd.setTextDatum(textdatum_t::top_right);
//   lcd.setTextDatum(textdatum_t::middle_left);
//   lcd.setTextDatum(textdatum_t::middle_center);
//   lcd.setTextDatum(textdatum_t::middle_right);
//   lcd.setTextDatum(textdatum_t::baseline_left);
//   lcd.setTextDatum(textdatum_t::baseline_center);
//   lcd.setTextDatum(textdatum_t::baseline_right);
//   lcd.setTextDatum(textdatum_t::bottom_left);
//   lcd.setTextDatum(textdatum_t::bottom_center);
//   lcd.setTextDatum(textdatum_t::bottom_right);
//   // "textdatum_t::" is optional
//   // For print functions, only vertical specifications are effective; horizontal specifications have no effect.

//   // Bottom Right Align
//   lcd.setTextDatum(bottom_right);
//   lcd.drawString("bottom_right", lcd.width() / 2, lcd.height() / 2);

//   // Bottom Left Align
//   lcd.setTextDatum(bottom_left);
//   lcd.drawString("bottom_left", lcd.width() / 2, lcd.height() / 2);

//   // Top Right Align
//   lcd.setTextDatum(top_right);
//   lcd.drawString("top_right", lcd.width() / 2, lcd.height() / 2);

//   // Top Left Align
//   lcd.setTextDatum(top_left);
//   lcd.drawString("top_left", lcd.width() / 2, lcd.height() / 2);


//   // Draw centerline in reference coordinate
//   lcd.drawFastVLine(lcd.width() / 2, 0, lcd.height(), 0xFFFFFFU);
//   lcd.drawFastHLine(0, lcd.height() / 2, lcd.width(), 0xFFFFFFU);


//   delay(3000);
//   drawGradation();

//   lcd.setFont(&Font2);
//   lcd.setCursor(0, 0);


//   lcd.drawRect(8, 8, lcd.width() - 16, lcd.height() - 16, 0xFFFFFFU);

//   // You can limit the drawing range with the setClipRect(). Drawing will not occur outside the specified range.
//   // It affects all drawing functions, not just text ones.
//   lcd.setClipRect(10, 10, lcd.width() - 20, lcd.height() - 20);


//   // Use the setTextSize() function to specify the text enlargement ratio.
//   // The first argument specifies the horizontal scale, and the second argument specifies the vertical scale.
//   // If the second argument is omitted, the scale of the first argument will be reflected in vertical and horizontal.
//   lcd.setTextSize(2.7, 4);
//   lcd.println("Size 2.7 x 4");

//   lcd.setTextSize(2.5);
//   lcd.println("Size 2.5 x 2.5");

//   lcd.setTextSize(1.5, 2);
//   lcd.println("Size 1.5 x 2");

//   delay(1000);

//   lcd.setTextColor(0xFFFFFFU, 0);
//   for (float i = 0; i < 30; i += 0.01) {
//     lcd.setTextSize(sin(i) + 1.1, cos(i) + 1.1);
//     lcd.drawString("size test", 10, 10);
//   }

//   lcd.setTextSize(1);

//   // The setTextWrap() specifies the wrapping behavior when the print() reaches the edge ​​of the drawing area.
//   // If the first argument is set to true, it will move to the left end after reaching the right end.
//   // If the second argument is set to true, it will move to the top after reaching the bottom. (Default: false)
//   lcd.setTextWrap(false);
//   lcd.println("setTextWrap(false) testing... long long long long string wrap test string ");
//   // If false is specified, wrapping will have no effect and out-of-bounds drawing will not occur.

//   lcd.setTextWrap(true);
//   lcd.setTextColor(0xFFFF00U, 0);
//   lcd.println("setTextWrap(true) testing... long long long long string wrap test string ");
//   // When true is specified, the coordinates will be automatically adjusted to fit within the drawing range.

//   delay(1000);

//   // If true is specified for the second argument, drawing will continue from the top
//   // when reaches to the bottom of the screen.
//   lcd.setTextColor(0xFFFFFFU, 0);
//   lcd.setTextWrap(true, true);
//   lcd.println("setTextWrap(true, true) testing...");
//   for (int i = 0; i < 100; ++i) {
//     lcd.printf("wrap test %03d ", i);
//     delay(50);
//   }


//   drawGradation();

//   // The setTextScroll() specifies the scrolling behavior when drawing reaches to the bottom of the screen.
//   // Use the setScrollRect() to specify the rectangular range to scroll (if not specified, the entire screen will scroll).
//   // The scrolling function requires that the LCD supports pixel readout.
//   lcd.setTextScroll(true);

//   // The first to fourth arguments specify the rectangular range of X, Y,
//   // Width and Height, and the fifth argument specifies the color after scrolling.
//   // The fifth argument is optional (no change when omitted).
//   lcd.setScrollRect(10, 10, lcd.width() - 20, lcd.height() - 20, 0x00001FU);

//   for (int i = 0; i < 50; ++i) {
//     lcd.printf("scroll test %d \n", i);
//   }


//   // Cancels the range specification of setClipRect().
//   lcd.clearClipRect();

//   // Cancels the range specification of setScrollRect().
//   lcd.clearScrollRect();


//   lcd.setTextSize(1);
//   lcd.setTextColor(0xFFFFFFU, 0);


//   // The setTextPadding() allows you to specify the minimum width
//   // to be used when filling the background with drawString functions.
//   lcd.setTextPadding(100);


//   drawGradation();
// }

// void drawNumberTest(const lgfx::IFont* font) {
//   lcd.setFont(font);

//   lcd.fillScreen(0x0000FF);

//   lcd.setColor(0xFFFF00U);
//   lcd.drawFastVLine(80, 0, 240);
//   lcd.drawFastVLine(160, 0, 240);
//   lcd.drawFastVLine(240, 0, 240);
//   lcd.drawFastHLine(0, 45, 320);
//   lcd.drawFastHLine(0, 95, 320);
//   lcd.drawFastHLine(0, 145, 320);
//   lcd.drawFastHLine(0, 195, 320);

//   for (int i = 0; i < 200; ++i) {
//     lcd.setTextDatum(textdatum_t::bottom_right);
//     lcd.drawNumber(i, 80, 45);
//     lcd.setTextDatum(textdatum_t::bottom_center);
//     lcd.drawNumber(i, 160, 45);
//     lcd.setTextDatum(textdatum_t::bottom_left);
//     lcd.drawNumber(i, 240, 45);
//     lcd.setTextDatum(textdatum_t::baseline_right);
//     lcd.drawNumber(i, 80, 95);
//     lcd.setTextDatum(textdatum_t::baseline_center);
//     lcd.drawNumber(i, 160, 95);
//     lcd.setTextDatum(textdatum_t::baseline_left);
//     lcd.drawNumber(i, 240, 95);
//     lcd.setTextDatum(textdatum_t::middle_right);
//     lcd.drawNumber(i, 80, 145);
//     lcd.setTextDatum(textdatum_t::middle_center);
//     lcd.drawNumber(i, 160, 145);
//     lcd.setTextDatum(textdatum_t::middle_left);
//     lcd.drawNumber(i, 240, 145);
//     lcd.setTextDatum(textdatum_t::top_right);
//     lcd.drawNumber(i, 80, 195);
//     lcd.setTextDatum(textdatum_t::top_center);
//     lcd.drawNumber(i, 160, 195);
//     lcd.setTextDatum(textdatum_t::top_left);
//     lcd.drawNumber(i, 240, 195);
//   }
// }

// void loop(void) {
//   // Fonts which names start with "Free" come in four sizes: 9pt, 12pt, 18pt, and 24pt.
//   drawNumberTest(&fonts::Font0);
//   drawNumberTest(&fonts::Font2);
//   drawNumberTest(&fonts::Font4);
//   drawNumberTest(&fonts::Font6);
//   drawNumberTest(&fonts::Font7);
//   drawNumberTest(&fonts::Font8);
//   drawNumberTest(&fonts::TomThumb);
//   drawNumberTest(&fonts::FreeMono9pt7b);
//   drawNumberTest(&fonts::FreeMonoBold9pt7b);
//   drawNumberTest(&fonts::FreeMonoOblique9pt7b);
//   drawNumberTest(&fonts::FreeMonoBoldOblique9pt7b);
//   drawNumberTest(&fonts::FreeSans9pt7b);
//   drawNumberTest(&fonts::FreeSansBold9pt7b);
//   drawNumberTest(&fonts::FreeSansOblique9pt7b);
//   drawNumberTest(&fonts::FreeSansBoldOblique9pt7b);
//   drawNumberTest(&fonts::FreeSerif9pt7b);
//   drawNumberTest(&fonts::FreeSerifBold9pt7b);
//   drawNumberTest(&fonts::FreeSerifItalic9pt7b);
//   drawNumberTest(&fonts::FreeSerifBoldItalic9pt7b);
//   drawNumberTest(&fonts::Orbitron_Light_24);
//   drawNumberTest(&fonts::Roboto_Thin_24);
//   drawNumberTest(&fonts::Satisfy_24);
//   drawNumberTest(&fonts::Yellowtail_32);
// }