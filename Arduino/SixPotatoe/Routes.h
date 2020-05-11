/*****************************************************************************-
 *                        Routes.h
 *****************************************************************************/


String go[] = { 
  "P     0,0    0         ",
  "G     0,1.5    3         ",
  "F"
};
String up[] = { 
  "P     0,0    0         ",
  "U 1 2",
  "F"
};
String turnL[] = { 
  "P      0,0    0         ",
  "G      0,3    3         ",
  "T       -300   0.2    3    ",
  "F"
};

/* Ramp edge is 7.2M from stand on wall
 * Ramp is 1.9M for the 90 degree curve  
 * ~5M to edge of rug.
 */
String bowl1[] = { 
  "P   0,0 0",
  "G 0,0.3  9",
  "G 0,7.25 14",
//  "G 0,7.25 3",
  "B 1.85       -5 -5 -5 -5 -5 -5 -5 -5 0 0 5 5 10 10 15 15 20 20 20 20 20",
  "A",
//  "Q",
  "B -1       -10 -10 -10 -10 -10 -10 -10 -10 0 0 0 0 0  0 0",
  "F"
};
String drop1[] = { 
  "P   0,0   0",
  "G   0,2   3",
  "F"
};
String drop2[] = { 
  "P   0,0   0",
  "G   0,2   6",
  "F"
};
String drop3[] = { 
  "P   0,0   0",
  "G   0,2   9",
  "F"
};

// 5 routes only
String* routeTable[] = {
  up,
  bowl1,
  drop1,
  drop2,
  drop3
};
 
