/*
 *
 * System5x7
 *
 *
 * File Name           : Clock5x7.h
 * Date                : 22.12.2018
 * Font size in bytes  : ??
 * Font width          : 5
 * Font height         : 7
 * Font first char     : 32
 * Font last char      : 39
 * Font used chars     : ....
 *
 * Use it with print(char(32))
 *
 * The font data are defined as
 *
 * struct _FONT_ {
 *     uint16_t   font_Size_in_Bytes_over_all_included_Size_it_self;
 *     uint8_t    font_Width_in_Pixel_for_fixed_drawing;
 *     uint8_t    font_Height_in_Pixel_for_all_characters;
 *     unit8_t    font_First_Char;
 *     uint8_t    font_Char_Count;
 *
 *     uint8_t    font_Char_Widths[font_Last_Char - font_First_Char +1];
 *                  // for each character the separate width in pixels,
 *                  // characters < 128 have an implicit virtual right empty row
 *
 *     uint8_t    font_data[];
 *                  // bit field of all characters
 */

#ifndef CLOCK5x7_H
#define CLOCK5x7_H

#define CLOCK5x7_WIDTH 5
#define CLOCK5x7_HEIGHT 7

GLCDFONTDECL(Clock5x7) = {
    0x0, 0x0, // size of zero indicates fixed width font,
    0x05, // width
    0x07, // height
    0x20, // first char 32
    0x0E, // char count 14
    
    // Fixed width; char width table not used !!!!
    
    // font data
    0x00, 0x00, 0x00, 0x00, 0x00,//  0 (space) char(32) 
	0x0F, 0x05, 0x7A, 0x10, 0x78,//  1 ! char(33) AM Symbol
	0x0E, 0x05, 0x7E, 0x10, 0x78,//  2 " char(34) PM Symbol
	0x7E, 0x43, 0x43, 0x43, 0x7E,//  3 # char(35) Battery Empty
	0x7E, 0x63, 0x63, 0x63, 0x7E,//  4 $ char(36) Battery 25%
	0x7E, 0x73, 0x73, 0x73, 0x7E,//  5 % char(37) Battery 50%
	0x7E, 0x7B, 0x7B, 0x7B, 0x7E,//  6 & char(38) Battery 75%
	0x7E, 0x7F, 0x7F, 0x7F, 0x7E,//  7 ' char(39) Battery 100%
	0x70, 0x7C, 0x7E, 0x7F, 0x7F,//  8 ( char(40) Upper Left Edge
	0x7F, 0x7F, 0x7E, 0x7C, 0x70,//  9 ) char(41) Upper Right Edge
	0x7F, 0x7F, 0x7F, 0x7F, 0x7F,// 10 * char(42) Full 5x7 Block
	0x07, 0x1F, 0x3F, 0x7F, 0x7F,// 11 + char(43) Lower Left Edge
	0x7F, 0x7F, 0x3F, 0x1F, 0x07,// 12 , char(44) Lower Right Edge
	0x38, 0x26, 0x61, 0x26, 0x38 // 13 - char(45) Bell Symbol
};

#endif
