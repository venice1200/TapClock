**2018-12-23**  
Clock5x7.h v0.2  
Added two new Characters for the BlockFont Number Zero  
  
**2018-12-23**  
Created a new font "Clock5x7.h" which includes all symbols from the changed "System5x7" font,  
new Symbols for an BlockClock Watchface and an Bell Symbol which looks like an Rocket.  

See https://i.imgur.com/vHU6aPZ.jpg  
Changes on "System5x7" can be reverted by the original file if you use the new Font file.  
  
You have two possibities to add the font to the sketch:
* Add the font to the SSD1306ASCii Library and add the line #include "Clock5x7.h" to the library file "allFonts.h" to cover the font. The font must be part of the SSD1306ASCii Library file and folder structure.
* Add the font to this (or your sketch) by adding an #include "Clock5x7.h" and make the file "Clock5x7.h" available through the libraries or sketch folders.  I keep it in the libraries\SSDFonts folder to use it more than once.

**2018-12-08**  
Replace the system5x7 font with the one here.  
Characters 123-127 showing a small battery,  
Character 91/93 very little PM/AM's symbols.
