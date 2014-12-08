//56b37c6f-792a-480f-b962-9a0db8c32aa4
/*
Ideas:
  Point to an object, identify it, and given the time it will give you your lat/long
  Zoom in to see text names, else maybe single letters (or symbols for planets)

Steps:
	Build Project Overview
	Figure out best data storage method
	

Programming Milestones:
 Draw Sphere Phase:
  DONE  Map Equatorial coordinate system to 2D screen
  DONE  Rotate with accelerometer to adjust view
        Rotate with compass to adjust view
        Render point data
        Render line data (constellations)
        Render grid
        Render only data in view
        Zoom Render
 Star Phase:
        Add Stars
        Render different magnitudes
        Constellations
        Labels
 Track Planets/Moon/Sun
 Ecliptic
 Messier Objects
 Menu System
 Calibrate Compass Screen
 
 
Label: Orion
 RA	  Dec
15241	654

 
 RA	Dec
13226	1620
13191	1265
	
13254	1019
13392	446
13587	313
14797	1156
15110	-55
14768	-434
14454	-1247
14315	-1493
15827	-1760
15509	-355
16164	1347
15252	1808
14797	1156
	
15509	-355
15301	-218
15110	-55
	
16164	1347
16492	1757
16729	2688
16128	3689
	
16492	1757
16927	2588
16562	3665
	
18248	1799
17881	1335
17466	837
18562	437
19654	-91
21901	-255
22237	-543
	
17881	1335
18562	437
	
21901	-255
20992	-1739
	
19654	-91
17695	-1280
17061	-1144
{{13226,1620},{13191,1265},{0,0},{13254,1019},{13392,446},{13587,313},{14797,1156},{15110,-55},{14768,-434},{14454,-1247},{14315,-1493},{15827,-1760},{15509,-355},{16164,1347},{15252,1808},{14797,1156},{0,0},{15509,-355},{15301,-218},{15110,-55},{0,0},{16164,1347},{16492,1757},{16729,2688},{16128,3689},{0,0},{16492,1757},{16927,2588},{16562,3665},{0,0},{18248,1799},{17881,1335},{17466,837},{18562,437},{19654,-91},{21901,-255},{22237,-543},{0,0},{17881,1335},{18562,437},{0,0},{21901,-255},{20992,-1739},{0,0},{19654,-91},{17695,-1280},{17061,-1144}}
 
*/


#include "pebble.h"
#define UPDATE_MS 50 // Refresh rate in milliseconds
//I don't know why I need to specify the Layer structure here; something changed in 2.6!
typedef struct Layer
{
  GRect bounds;
  GRect frame;
  bool clips : 1;
  bool hidden : 1;
  struct Layer *next_sibling;
  struct Layer *parent;
  struct Layer *first_child;
  struct Window *window;
  LayerUpdateProc update_proc;
} Layer;

static Window *main_window;
static Layer *sky_layer, *popup_layer;
typedef struct popupStruct {
  int8_t exists;
  int16_t height;
  int16_t maxheight;
  int16_t speed;
  int16_t time;
  int16_t duration;
  char *msg;//[200];
} popupStruct;
popupStruct popup = {0, 0, 30, 1, 0, 0, ""};

typedef struct PointStruct {
  int16_t dec;   // declination
  int16_t ra;    // right ascension
} PointStruct;
PointStruct lines[5000];

typedef struct CamStruct {
  int16_t pitch;   // up/down [-16384, 16384]
  int16_t roll;    // aileron roll [-32768, 32768]
  int16_t yaw;     // direction facing [-32768, 32768]
} CamStruct;
CamStruct cam;

int32_t T0=0, T1=0, T2=0, T3=0, T4=0, T5=0;
int8_t zoom=8;
uint16_t maxdots=0;
// ------------------------------------------------------------------------ //
//  Helper Functions
// ------------------------------------------------------------------------ //
#define root_depth 20          // How many iterations square root function performs
int32_t sqrt32(int32_t a) {int32_t b=a; for(uint8_t i=0; i<root_depth; i++) b=(b+(a/b))/2; return b;} // Square Root 32bit
 int8_t   abs8( int8_t x) {return (x ^ (x >>  7)) - (x >>  7);}
int16_t  abs16(int16_t x) {return (x ^ (x >> 15)) - (x >> 15);}
int32_t  abs32(int32_t x) {return (x ^ (x >> 31)) - (x >> 31);}

// ------------------------------------------------------------------------ //
//  Program Functions
// ------------------------------------------------------------------------ //
void init_dots() {
  for(int32_t declination=-16384; declination<=16384; declination+=256) // More Lattitudes
  //for(int32_t declination=-16384; declination<=16384; declination+=2048) // All Lattitudes
  //for(int32_t declination=0; declination<=16384; declination+=2048) // Just Horizon and Upper Lattitudes
    
    //for(int32_t right_ascension=0; right_ascension<TRIG_MAX_ANGLE; right_ascension+=512) {  // More Lines
    //for(int32_t right_ascension=0; right_ascension<TRIG_MAX_ANGLE; right_ascension+=2048) {  // All Lines
    //for(int32_t right_ascension=0; right_ascension<TRIG_MAX_ANGLE; right_ascension+=2048*4) { // Less Lines
    for(int32_t right_ascension=0; right_ascension<2048; right_ascension+=2048) { // Just north line
    
      dots[maxdots].dec = (declination+TRIG_MAX_ANGLE)%TRIG_MAX_ANGLE;
      dots[maxdots].ra  = right_ascension;
      maxdots++;
    if(maxdots==5000) maxdots--;
      //TODO: Change to be 2 halves of a 32bit number: dots[n++] = declination << 16 & right_ascention
    }
  
    //for(int32_t right_ascension=0; right_ascension<TRIG_MAX_ANGLE; right_ascension+=1024) {dots[maxdots].dec=0; dots[maxdots].ra=right_ascension; maxdots++;} // Lotsa Horizon Dots
  for(int32_t right_ascension=0; right_ascension<TRIG_MAX_ANGLE; right_ascension+=256) {dots[maxdots].dec=0; dots[maxdots].ra=right_ascension; maxdots++;} // Lotsa Horizon Dots
  
  
  APP_LOG(APP_LOG_LEVEL_INFO, "maxdots = %d", maxdots);
}

static AccelData accel;// = {0, 0, 0};

void update_camera() {
  //static AccelData accel; accel_service_peek(&accel);                          // Read accelerometer
  accel_service_peek(&accel);                          // Read accelerometer

  static CompassHeadingData heading_data; compass_service_peek(&heading_data); // Read magnetometer
  int16_t angle;
  

  //if(accel.z > 1024) accel.z = 1024; if(accel.z <-1024) accel.z =-1024;      // -1024 and 1024 are top and bottom

  //angle = TRIG_MAX_ANGLE - atan2_lookup(accel.z, -1 * accel.y);
  //angle  = TRIG_MAX_ANGLE - (accel.z*16);                              // real-time pitch (jittery)
  //cam.pitch += ((angle - cam.pitch) / 8) + ((angle - cam.pitch) % 8);  // low-pass filter to reduce jitters
  
  //angle  = TRIG_MAX_ANGLE - atan2_lookup(accel.x, -1 * accel.y);       // Real-time roll (jittery)
  //cam.roll  += ((angle - cam.roll) / 8) + ((angle - cam.roll) % 8);    // low-pass filter to reduce jitters
  
  //cam.yaw += cam.roll / 1024; // Compensate for Compass being dumb
  //if(abs16(angle)<1000)   // Only read compass if pebble is in portrait orientation
    //cam.yaw += ((int16_t)(heading_data.true_heading - cam.yaw))/8 + ((int16_t)(heading_data.true_heading - cam.yaw))%8;

  // raw data
  cam.pitch = TRIG_MAX_ANGLE - (accel.z*16);
  //cam.pitch = TRIG_MAX_ANGLE - atan2_lookup(accel.z, -1 * accel.y);
  cam.roll  = TRIG_MAX_ANGLE - atan2_lookup(accel.x, -1 * accel.y);
  cam.yaw   = heading_data.true_heading;
}

void update_popup() {
  if(popup.exists) {
    if(popup.time == 0) {
      if(popup.height < popup.maxheight)
        popup.height+=popup.speed;
      else 
        popup.time++;
    } else if(popup.time == popup.duration) {
      if(popup.height < 1)
        popup.exists = 0;
      else
        popup.height-=popup.speed;
    } else {
      popup.time++;
    }
    layer_set_frame(popup_layer, GRect(0, 168-popup.height, 144, popup.maxheight));
  }
  
}

void popup_message(char *msg, int16_t maxheight, int16_t duration) {
  if(popup.exists == false) {
    popup.exists    = 1;
    popup.time      = 0;
    popup.height    = 0;
    popup.speed     = 2;
    popup.maxheight = maxheight;
    popup.msg       = msg;
    popup.duration  = duration;
  }
}

// ------------------------------------------------------------------------ //
//  Timer Functions
// ------------------------------------------------------------------------ //
static void timer_callback(void *data) {
  update_camera();
  update_popup();
  layer_mark_dirty(sky_layer);  // Schedule redraw of screen
  layer_mark_dirty(popup_layer);  // Schedule redraw of screen
}

/*  Saving Backup Copy of Previous Attempts
static void timer_callback(void *data) {
  static AccelData accel; accel_service_peek(&accel);                          // Read accelerometer
  static CompassHeadingData heading_data; compass_service_peek(&heading_data); // Read magnetometer
  
  //Version 1: Raw Accelerometer Data -- Jittery
  // CAM_PITCH = TRIG_MAX_ANGLE - atan2_lookup(accel.z, -1 * accel.y);
  // CAM_ROLL = TRIG_MAX_ANGLE - atan2_lookup(accel.x, -1 * accel.y);
  // CAM_YAW = heading_data.true_heading;

  //Version 2: Smooth Jitters with Moving Average
  //Moving Average Equation: X = ((X*7) + Y) / 8;
  // CAM_PITCH = ((CAM_PITCH * 3) + ((int32_t)TRIG_MAX_ANGLE - (int32_t)atan2_lookup(accel.z, -1 * accel.y))) / 4;
  // CAM_ROLL  = ((CAM_ROLL  * 3) + ((int32_t)TRIG_MAX_ANGLE - (int32_t)atan2_lookup(accel.x, -1 * accel.y))) / 4;
  // CAM_YAW   = ((CAM_YAW   * 3) + (int32_t)heading_data.true_heading) / 4;

  // Version 3: Smooth Jitters with Low-Pass Filter
  //Low-Pass Filter: X = X + Difference / (Part To Move)
  //CurrentPosition = CAM_PITCH;
  //AimingPosition = TRIG_MAX_ANGLE - atan2_lookup(accel.z, -1 * accel.y);
  //Difference = (int32_t)((int16_t)(AimingPosition - CurrentPos));
  //NewPosition = CurrentPos + (Difference / 8);
  CAM_PITCH += (int32_t)((int16_t)( (TRIG_MAX_ANGLE - atan2_lookup(accel.z, -1 * accel.y)) - CAM_PITCH))/8;
  CAM_ROLL  += (int32_t)((int16_t)( (TRIG_MAX_ANGLE - atan2_lookup(accel.x, -1 * accel.y)) - CAM_ROLL ))/8;
  CAM_YAW   += (int32_t)((int16_t)( heading_data.true_heading - CAM_YAW))/8;
  
  layer_mark_dirty(graphics_layer);  // Schedule redraw of screen
}
*/

// ------------------------------------------------------------------------ //
//  Button Functions
// ------------------------------------------------------------------------ //
void up_single_click_handler(ClickRecognizerRef recognizer, void *context) {zoom++;}

void select_single_click_handler(ClickRecognizerRef recognizer, void *context) {
  popup_message("hello", 30, 20); // msg, height, duration
}

void down_single_click_handler(ClickRecognizerRef recognizer, void *context) {
      //dots[maxdots].dec = TRIG_MAX_ANGLE - cam.pitch;
      //dots[maxdots].ra  = TRIG_MAX_ANGLE - cam.yaw;
      //maxdots++;
  zoom--;
}
  
void click_config_provider(void *context) {
  window_single_click_subscribe(BUTTON_ID_UP, up_single_click_handler);
  window_single_click_subscribe(BUTTON_ID_SELECT, select_single_click_handler);
  window_single_click_subscribe(BUTTON_ID_DOWN, down_single_click_handler);
}

// ------------------------------------------------------------------------ //
//  Drawing Functions
// ------------------------------------------------------------------------ //
static void sky_layer_update(Layer *me, GContext *ctx) {
  GPoint point=(GPoint){.x=0, .y=0};
  int32_t x, y, z, z1;
  int16_t ra, dec;
  int32_t cos1, cos2, sin2;
  
  time_t sec1, sec2; uint16_t ms1, ms2; int32_t dt; // time snapshot variables, to calculate render time and FPS
  
  graphics_context_set_stroke_color(ctx, 1);
  time_ms(&sec1, &ms1);  //1st Time Snapshot
  for(int16_t i=0; i<maxdots; i++) {
    ra  = dots[i].ra + cam.yaw;
    y   = sin_lookup(dots[i].dec + (TRIG_MAX_ANGLE/2)) >> 8;
    cos1 = cos_lookup(dots[i].dec) >> 8;
    z   = (cos1 * (cos_lookup(ra) >> 8)) >> 8;

    cos2 = cos_lookup(cam.pitch)>>8;
    sin2 = sin_lookup(cam.pitch)>>8;
    z1  = y*sin2 + z*cos2;

    if(z1 > 0) {  // If in front of camera
      y   = (y*cos2 - z*sin2)>>8;
      x   = (cos1 * (sin_lookup(ra) >> 8)) >> 8;
      
      cos2 = cos_lookup(cam.roll)>>8;
      sin2 = sin_lookup(cam.roll)>>8;
            
      point.x = (( (x*cos2 - y*sin2) << zoom) / z1) + 72;
      point.y = (( (x*sin2 + y*cos2) << zoom) / z1) + 84;
      
      if(point.x>=0 && point.x<me->frame.size.w && point.y>=0 && point.y<me->frame.size.h)  // If within screen bounds
        graphics_draw_circle(ctx, point, zoom-5);
    }
    
  }
  time_ms(&sec2, &ms2);  //2nd Time Snapshot
  
  dt = ((int32_t)1000*(int32_t)sec2 + (int32_t)ms2) - ((int32_t)1000*(int32_t)sec1 + (int32_t)ms1);  //ms between two time snapshots
  static char textbuffer[200];  //Buffer to hold text
  GRect textframe = GRect(0, 0, 144, 168);  // Text Box Position and Size: x, y, w, h
  //ra  = cam.yaw/-180;//(TRIG_MAX_ANGLE - cam.yaw  ) / 360;
  //dec = cam.pitch/-180;//(TRIG_MAX_ANGLE/2 - cam.pitch) / 360;
  
  ra  = (((((cam.yaw  >>9)*-360)>>7))+360)%360;  //(TRIG_MAX_ANGLE - cam.yaw  ) / 360;
  dec =     ((cam.pitch>>9)*-360)>>7;   //(TRIG_MAX_ANGLE/2 - cam.pitch) / 360;
  
  snprintf(textbuffer, sizeof(textbuffer), "%ldms\n ra = %dº\ndec= %dº\n\nx:%d\ny:%d\nz:%d", dt, ra, dec, accel.x>>3, accel.y>>3, accel.z>>3);
  graphics_context_set_text_color(ctx, 1);  // Text Color
  graphics_draw_text(ctx, textbuffer, fonts_get_system_font(FONT_KEY_GOTHIC_14), textframe, GTextOverflowModeWordWrap, GTextAlignmentLeft, NULL);  //Write Text
  
  app_timer_register(UPDATE_MS, timer_callback, NULL); // Schedule a callback
}
/*
static void sky_layer_update(Layer *me, GContext *ctx) {
  GPoint point=(GPoint){.x=0, .y=0};
  //int32_t x, y, z, tmp;
  int32_t x1, y1, z1;
  int32_t y2, z2;
  int16_t ra, dec;
  int32_t cos1, cos2, sin2;
  
  
  time_t sec1, sec2; uint16_t ms1, ms2; int32_t dt; // time snapshot variables, to calculate render time and FPS
  
  graphics_context_set_stroke_color(ctx, 1);
  time_ms(&sec1, &ms1);  //1st Time Snapshot
  for(int16_t i=0; i<maxdots; i++) {
    //z=sin(ra)*cos(dec)
    //x=sin(ra)*sin(dec)
    //y=cos(ra)

    //http://en.wikipedia.org/wiki/Ecliptic_coordinate_system
    //x = cos(dec) * cos(ra)
    //y = cos(dec) * sin(ra)
    //z = sin(dec)
    
    //dec = dots[i].dec + cam.pitch;
//ra = dec + 1;
    //ra = dots[i].ra;// + cam.yaw;
    
    //z1 = (cos1 * (cos_lookup(ra) >> 8)) >> 8;
    //x1 = cos1 * (sin_lookup(ra) >> 8) >> 8;
    //y1 = sin_lookup(dots[i].dec + (TRIG_MAX_ANGLE/2)) >> 8;

    //cos2 = cos_lookup(cam.pitch)>>8;
    //sin2 = sin_lookup(cam.pitch)>>8;
    //y2 = (y1*cos2 - z1*sin2)>>8;
    //z2 =  y1*sin2 + z1*cos2;
    
    
    
    
    //z1 = ((cos_lookup(dots[i].dec) >> 8) * (cos_lookup(ra) >> 8)) >> 8;
    //x1 = ((cos_lookup(dots[i].dec) >> 8) * (sin_lookup(ra) >> 8)) >> 8;
    //y1 = sin_lookup(dots[i].dec + (TRIG_MAX_ANGLE/2)) >> 8;
    
    
    
    ra = dots[i].ra + cam.yaw;
    dec = dots[i].dec + cam.pitch;
    z1 = (cos_lookup(dec) >> 8) * (cos_lookup(ra) >> 8);
    x1 = (cos_lookup(dec) >> 8) * (sin_lookup(ra) >> 8);
    y1 = sin_lookup(dec + (TRIG_MAX_ANGLE/2));

    //y >>= 8; z >>= 8; cos = cos_lookup(cam.pitch)>>8; sin = sin_lookup(cam.pitch)>>8;
    //tmp = y*cos - z*sin; z = y*sin + z*cos; y = tmp;

    if(z1>0) { // If in front of camera
      x1 >>= 8; y1 >>= 8;
      cos2 = cos_lookup(cam.roll)>>8; sin2 = sin_lookup(cam.roll)>>8;
      point.x = ((((x1*cos2 - y1*sin2)<<zoom) / z1)) + 72;
      point.y = ((((x1*sin2 + y1*cos2)<<zoom) / z1)) + 84;
            
      //x1 = (x1<<zoom) / z2;
      //y2 = (y2<<zoom) / z2;
      
      //if(x1>-84 && x1<84 && y2<-84 && y2>84) {
      //cos2 = cos_lookup(cam.roll)>>8;
      //sin2 = sin_lookup(cam.roll)>>8;
      
      //point.x = (((x1*cos2 - y1*sin2)<<zoom) / z1) + 72;
      //point.y = (((x1*sin2 + y1*cos2)<<zoom) / z1) + 84;
      
      
      //point.x = (x1*cos2 - y2*sin2) + 72;
      //point.y = (x1*sin2 + y2*cos2) + 84;
       
      if(point.x>=0 && point.x<me->frame.size.w && point.y>=0 && point.y<me->frame.size.h)
        graphics_draw_circle(ctx, point, 3);
     // }
    }
  }
  time_ms(&sec2, &ms2);  //2nd Time Snapshot
  
  dt = ((int32_t)1000*(int32_t)sec2 + (int32_t)ms2) - ((int32_t)1000*(int32_t)sec1 + (int32_t)ms1);  //ms between two time snapshots
  static char textbuffer[100];  //Buffer to hold text
  GRect textframe = GRect(0, 0, 144, 30);  // Text Box Position and Size: x, y, w, h
  snprintf(textbuffer, sizeof(textbuffer), "%ldms", dt);
  graphics_context_set_text_color(ctx, 1);  // Text Color
  graphics_draw_text(ctx, textbuffer, fonts_get_system_font(FONT_KEY_GOTHIC_14), textframe, GTextOverflowModeWordWrap, GTextAlignmentLeft, NULL);  //Write Text
  
  app_timer_register(UPDATE_MS, timer_callback, NULL); // Schedule a callback
}
*/
/*
static void sky_layer_update(Layer *me, GContext *ctx) {
  GPoint point=(GPoint){.x=0, .y=0};
  GPoint point2=(GPoint){.x=0, .y=0};
  //APP_LOG(APP_LOG_LEVEL_INFO, "ra=%d dec=%d = %d %d", dots[250].ra, dots[250].dec, point.x, point.y);
  int32_t x, y, z, tmp;
  //int32_t x2, y2, z2;
  int32_t cos, sin, ra;
  time_t sec1, sec2; uint16_t ms1, ms2; int32_t dt; // time snapshot variables, to calculate render time and FPS
  
  graphics_context_set_stroke_color(ctx, 1);
  time_ms(&sec1, &ms1);  //1st Time Snapshot
  for(int16_t i=0; i<maxdots; i++) {
    // Convert Spherical Coordinates to XYZ
    //http://en.wikipedia.org/wiki/Spherical_coordinate_system
    //z=sin(ra)*cos(dec) x=sin(ra)*sin(dec) y=cos(ra)  // these might not be right?

    //http://en.wikipedia.org/wiki/Ecliptic_coordinate_system
    //x = cos(dec) * cos(ra)
    //y = cos(dec) * sin(ra)
    //z = sin(dec)

    //2D_point.x = 3D_point.x / 3D_point.z;  2D_point.y = 3D_point.y / 3D_point.z;
    //     X-axis                    Y-axis                     Z-axis
    //x' = x                     x' = z sin(a) + x cos(a)   x' = x cos(a) - y sin(a)
    //y' = y cos(a) - z sin(a)   y' = y                     y' = x sin(a) + y cos(a)
    //z' = y sin(a) + z cos(a)   z' = z cos(a) - x sin(a)   z' = z
    
    // All these ">>8" are cause TRIG_MAX_ANGLE is multiplied by each lookup.
    //   TRIG_MAX_ANGLE = 1 << 16, so splitting the divide between both
    //   Could just compute the answer THEN >>16, but answer might overflow.
    // Z and X had 2 TRIG_MAX_ANGLES and Y only one.  After >>8's all now have one.
    
    // Rotate Yaw
    ra = dots[i].ra + cam.yaw;
    
    // Get X, Y and Z
    cos = cos_lookup(dots[i].dec) >> 8;
    z = cos * (cos_lookup(ra) >> 8);
    x = cos * (sin_lookup(ra) >> 8);
    //y = -1 * sin_lookup(dots[i].dec);               // Invert due to y+ = downward on LCD
    y = sin_lookup(dots[i].dec + (TRIG_MAX_ANGLE/2)); // Invert due to y+ = downward on LCD.  sin(x+pi) = -1 * sin(x)

    // Rotate Pitch
    y >>= 8;
    z >>= 8;
    cos = cos_lookup(cam.pitch)>>8;
    sin = sin_lookup(cam.pitch)>>8;
    tmp = y*cos - z*sin;
    z   = y*sin + z*cos;
    y   = tmp;

    // Rotate Roll
    x >>= 8;
    y >>= 8;
    cos = cos_lookup(cam.roll)>>8;
    sin = sin_lookup(cam.roll)>>8;
    tmp = x*cos - y*sin;
    y = x*sin + y*cos;
    x = tmp;
    
    if(z>0) { // If in front of camera
      //point.x = (((x<<8) / z)) + 72;  point.y = (((y<<8) / z)) + 84; // THIS <<8 is just the zoom factor
      point.x = (((x<<zoom) / z)) + 72;
      point.y = (((y<<zoom) / z)) + 84;
      if(point.x >= 0  &&  point.x < me->frame.size.w  &&  point.y >= 0  &&  point.y < me->frame.size.h)  // If within screen bounds
        graphics_draw_circle(ctx, point, 3);
        //graphics_draw_pixel(ctx, point);
      
      //if(point.x >= 0  &&  point.x < me->frame.size.w  &&  point.y >= 0  &&  point.y < me->frame.size.h) { // If within screen bounds
      //if(point2.x==0 && point2.y==0) point2=point;
      //graphics_draw_line(ctx, point, point2);
      //point2=point;
      //}
    } else {
      //point2=(GPoint){0,0};
    }
  }
  time_ms(&sec2, &ms2);  //2nd Time Snapshot
  
  graphics_context_set_stroke_color(ctx, 1);  graphics_draw_rect(ctx, me->frame); //White Border Around Frame
  
  static char textbuffer[100];  //Buffer to hold text
  GRect textframe = GRect(0, 0, 144, 30);  // Text Box Position and Size: x, y, w, h
  //snprintf(textbuffer, sizeof(textbuffer), " Hello 0x%x", (unsigned int)ctx);  // What text to draw
  //snprintf(textbuffer, sizeof(textbuffer), "%ld %ld %ld %ld", CAM_PITCH, CAM_ROLL, CAM_YAW);  // What text to draw
  T0=popup.exists;
  T1=popup.time;
  T2=popup.height;
  T3=popup.speed;
  T4=popup.duration;
  T5=popup.maxheight;
  snprintf(textbuffer, sizeof(textbuffer), "%+06ld %+06ld %+06ld %+06ld %+06ld %+06ld", T0, T1, T2, T3, T4, T5);  // What text to draw
  graphics_context_set_fill_color(ctx, 0); graphics_fill_rect(ctx, textframe, 0, GCornerNone);  //Black Filled Rectangle
  graphics_context_set_stroke_color(ctx, 1); graphics_draw_rect(ctx, textframe);                //White Rectangle Border
  graphics_context_set_text_color(ctx, 1);  // Text Color
  graphics_draw_text(ctx, textbuffer, fonts_get_system_font(FONT_KEY_GOTHIC_14), textframe, GTextOverflowModeWordWrap, GTextAlignmentLeft, NULL);  //Write Text


  
  dt = ((int32_t)1000*(int32_t)sec2 + (int32_t)ms2) - ((int32_t)1000*(int32_t)sec1 + (int32_t)ms1);  //ms between two time snapshots
  static char textbuffer[100];  //Buffer to hold text
  GRect textframe = GRect(0, 0, 144, 30);  // Text Box Position and Size: x, y, w, h
  snprintf(textbuffer, sizeof(textbuffer), "%ldms", dt);  // What text to draw
  graphics_context_set_fill_color(ctx, 0); graphics_fill_rect(ctx, textframe, 0, GCornerNone);  //Black Filled Rectangle
  graphics_context_set_stroke_color(ctx, 1); graphics_draw_rect(ctx, textframe);                //White Rectangle Border
  graphics_context_set_text_color(ctx, 1);  // Text Color
  graphics_draw_text(ctx, textbuffer, fonts_get_system_font(FONT_KEY_GOTHIC_14), textframe, GTextOverflowModeWordWrap, GTextAlignmentLeft, NULL);  //Write Text

  
  app_timer_register(UPDATE_MS, timer_callback, NULL); // Schedule a callback
}
*/
static void popup_layer_update(Layer *me, GContext *ctx) {  
  //static char textbuffer[100];  //Buffer to hold text
  if(popup.exists) {
    //layer_set_hidden(me, false);
    //layer_set_frame(me, GRect(0, 168-popup.height, 144, popup.maxheight));
    
    GRect textframe = GRect(0, 0, 144, popup.maxheight);  // Text Box Position and Size: x, y, w, h
  //snprintf(textbuffer, sizeof(textbuffer), " Hello 0x%x", (unsigned int)ctx);  // What text to draw
  //snprintf(textbuffer, sizeof(textbuffer), "%ld %ld %ld %ld", CAM_PITCH, CAM_ROLL, CAM_YAW);  // What text to draw
    //snprintf(textbuffer, sizeof(textbuffer), "%+06ld %+06ld %+06ld %+06ld %+06ld %+06ld", T0, T1, T2, T3, T4, T5);  // What text to draw
    graphics_context_set_fill_color(ctx, 0); graphics_fill_rect(ctx, textframe, 0, GCornerNone);  //Black Filled Rectangle
    graphics_context_set_stroke_color(ctx, 1); graphics_draw_rect(ctx, textframe);                //White Rectangle Border
    graphics_context_set_text_color(ctx, 1);  // Text Color
    graphics_draw_text(ctx, popup.msg, fonts_get_system_font(FONT_KEY_GOTHIC_14), textframe, GTextOverflowModeWordWrap, GTextAlignmentLeft, NULL);  //Write Text
  }
}
  
// ------------------------------------------------------------------------ //
//  Main Functions
// ------------------------------------------------------------------------ //
static void main_window_load(Window *window) {
  Layer *window_layer = window_get_root_layer(window);

  //sky_layer = layer_create(layer_get_frame(window_layer));
  sky_layer = layer_create(GRect(0, 0, 144, 168));
  layer_set_update_proc(sky_layer, sky_layer_update);
  layer_add_child(window_layer, sky_layer);

  popup_layer = layer_create(GRect(0, 167, 144, 30));
  layer_set_update_proc(popup_layer, popup_layer_update);
  layer_add_child(window_layer, popup_layer);
}

static void main_window_unload(Window *window) {
  layer_destroy(sky_layer);
  layer_destroy(popup_layer);
  // Destroy all layers of the window here
}

static void main_window_appear(Window *window) {}
static void main_window_disappear(Window *window) {}

static void init(void) {
  // Set up and push main window
  main_window = window_create();
  window_set_window_handlers(main_window, (WindowHandlers) {.load = main_window_load, .unload = main_window_unload});
  window_set_click_config_provider(main_window, click_config_provider);
  window_set_background_color(main_window, 0);
  window_set_fullscreen(main_window, true);

  //Set up other stuff
  srand(time(NULL));  // Seed randomizer
  accel_data_service_subscribe(0, NULL);  // We will be using the accelerometer
  init_dots();
  cam.pitch=0; cam.roll=0; cam.yaw=0;  // Init Camera  (Maybe initialize to actual accelerometer/magnetometer data)
  
  //Begin
  window_stack_push(main_window, true /* Animated */); // Display window.  Callback will be called once layer is dirtied then written
}
  
static void deinit(void) {
  accel_data_service_unsubscribe();
  window_destroy(main_window);
}

int main(void) {
  init();
  app_event_loop();
  deinit();
}

// ------------------------------------------------------------------------ //
//  Notes
// ------------------------------------------------------------------------ //
/*
#define FONT_KEY_FONT_FALLBACK "RESOURCE_ID_FONT_FALLBACK"
#define FONT_KEY_GOTHIC_14 "RESOURCE_ID_GOTHIC_14"
#define FONT_KEY_GOTHIC_14_BOLD "RESOURCE_ID_GOTHIC_14_BOLD"
#define FONT_KEY_GOTHIC_18 "RESOURCE_ID_GOTHIC_18"
#define FONT_KEY_GOTHIC_18_BOLD "RESOURCE_ID_GOTHIC_18_BOLD"
#define FONT_KEY_GOTHIC_24 "RESOURCE_ID_GOTHIC_24"
#define FONT_KEY_GOTHIC_24_BOLD "RESOURCE_ID_GOTHIC_24_BOLD"
#define FONT_KEY_GOTHIC_28 "RESOURCE_ID_GOTHIC_28"
#define FONT_KEY_GOTHIC_28_BOLD "RESOURCE_ID_GOTHIC_28_BOLD"
#define FONT_KEY_BITHAM_30_BLACK "RESOURCE_ID_BITHAM_30_BLACK"
#define FONT_KEY_BITHAM_42_BOLD "RESOURCE_ID_BITHAM_42_BOLD"
#define FONT_KEY_BITHAM_42_LIGHT "RESOURCE_ID_BITHAM_42_LIGHT"
#define FONT_KEY_BITHAM_42_MEDIUM_NUMBERS "RESOURCE_ID_BITHAM_42_MEDIUM_NUMBERS"
#define FONT_KEY_BITHAM_34_MEDIUM_NUMBERS "RESOURCE_ID_BITHAM_34_MEDIUM_NUMBERS"
#define FONT_KEY_BITHAM_34_LIGHT_SUBSET "RESOURCE_ID_BITHAM_34_LIGHT_SUBSET"
#define FONT_KEY_BITHAM_18_LIGHT_SUBSET "RESOURCE_ID_BITHAM_18_LIGHT_SUBSET"
#define FONT_KEY_ROBOTO_CONDENSED_21 "RESOURCE_ID_ROBOTO_CONDENSED_21"
#define FONT_KEY_ROBOTO_BOLD_SUBSET_49 "RESOURCE_ID_ROBOTO_BOLD_SUBSET_49"
#define FONT_KEY_DROID_SERIF_28_BOLD "RESOURCE_ID_DROID_SERIF_28_BOLD"





*/