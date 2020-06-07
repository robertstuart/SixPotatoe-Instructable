/*****************************************************************************-
 *                        Routes.h
 *****************************************************************************/


String LittleP[] = { 
  "P     0,0    0       ",
  "G     0,1         3  ",
  "T    90     0.5   4  ",
  "G   2.5,1.5          ",
  "T    90     0.5      ",
  "G     3,0            ",
  "T   180     0.5      ",
  "G     2,1            ",
  "T   -90     0.5      ",
  "G   0.5,1.5          ",
  "T   -90     0.5      ",
  "G     0,0.2          ",   
  "F"
};
String Fat8[] = { 
  "P     3,3    0        ",
  "T   180      1.5  6   ",
  "G     6,1.5           ",
  "T   180      1.5      ",
  "G     3,3             ",
  "T  -180      1.5      ",
  "G     0,1.5           ",
  "T  -180      1.5      ",
  "G     3,3             ", 
  "F"
};
String LittleSquare[] = { 
  "P     0,0    0        ",
  "G     0,1         5   ",
  "T    90      .5       ",
  "G     1,1.5           ",
  "T    90      .5       ",
  "G   1.5,.5            ",
  "T    90      .5       ",
  "G    .5,0             ",
  "T    90      .5       ",
  "G     0,1             ",
  "F"
};
String Dance[] = { 
  "P      0,0    0         ",
  "G     0,1.7       6  ",
  "T     180    0.2     ",
  "T    -180    0.2     ",
  "T     180    0.2     ",
  "T    -180    0.2     ",
  "T     180    0.2     ",
  "T    -180    0.2     ",
  "T     180    0.2     ",
  "T      90    0.2     ",
  "T    -180    0.2     ",
  "T     180    0.2     ",
  "T    -180    0.2     ",
  "T     180    0.2     ",
  "T    -180    0.2     ",
  "T     180    0.2     ",
  "F"
};
String BigOutNBack1[] = { 
  "P      0,0    0      ",
  "G     0,1         6  ",
  "G     0,40       12  ",
  "T    180   2         ",
  "T     90   2         ",
  "T    -90   1.7       ",
  "G     0,5            ",
  "G     0,1         7  ",
  "G     0,0         4  ",
  "F"
};
String BigOutNBack2[] = { 
  "P      0,0    0      ",
  "G     0,1         6  ",
  "G     0,40       12  ",
  "T   -180   2         ",
  "T    -90   2         ",
  "T     90   1.7       ",
  "G     0,5            ",
  "G     0,1         7  ",
  "G     0,0         4  ",
  "F"
};


// 5 routes: 1, 2, 3, 4, & 5;
String* routeTable[] = {
  LittleP,
  Fat8,
  LittleSquare,
  Dance,
  BigOutNBack1
};
 
