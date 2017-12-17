#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ncurses.h>
#include <libfreenect.h>
#include <stdarg.h>

#include <assert.h>
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/extensions/XTest.h>

#include <pthread.h>

#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <math.h>

#include <XnCore.h>

#include "dbg.h" // Debug macros

#define SCREEN (DefaultScreen(display))

// Defines frame size for the camera
#define FREENECT_FRAME_W 640
#define FREENECT_FRAME_H 480
#define FREENECT_FRAME_PIX (FREENECT_FRAME_H*FREENECT_FRAME_W)

// Defines frame size for the IR cam
#define FREENECT_IR_FRAME_W 640
#define FREENECT_IR_FRAME_H 480
#define FREENECT_IR_FRAME_PIX (FREENECT_IR_FRAME_H*FREENECT_IR_FRAME_W)

#define FREENECT_VIDEO_RGB_SIZE (FREENECT_FRAME_PIX*3)

// Defines console rows information
#define CONSOLE_MAX_ROWS 24
#define CONSOLE_ROWS_HEIGHT 8.5
#define MAX_OUT_BUFFER_ROWS 23

typedef struct console_info{
  int Sensitivity;
  int Rgb;
  int Depth;
  freenect_led_options LED;
  int Angle;
  int DepthFrameX1, DepthFrameX2, DepthFrameY1, DepthFrameY2, RgbFrameX1, RgbFrameX2, RgbFrameY1, RgbFrameY2;

  // input / output
  char *Buf;
  char conIn[1000];
  size_t len;
  char *OutBuf[MAX_OUT_BUFFER_ROWS];

  // Rows position
  int Rows[CONSOLE_MAX_ROWS];
} console;

typedef enum FEEDS{
  DEPTH,
  RGB
} FEED;

typedef enum LOGLEVEL{
  INFO,
  DEBUG
} LEVEL;


/*
  Purpose to eventually handle multiple Kinect at the same time, but not worried about that right now.
  Hold information to handle kinect.
*/
typedef struct connectedKinect{
  int nr_devices;
  int user_device_number; //Do I need this, from old code.
  int freenect_is_init; //0 = true
  int kinect_is_open; // 0 = true
  const char *kinect_serial;
  struct freenect_device_attributes *kinect_attributes;
  int kinect_supported_devices_count, kinect_selected_devices_count, kinect_selected_devices_flag;
  freenect_device_flags kinect_supported_devices[3];
  freenect_device_flags kinect_selected_devices[3];
}MYKINECT;



//// BEGIN FUNCTION DECLARATION

/**
 * Function to play with kinectSensor
 */
void runTest();
/**
 * List selected (Called enabled by libFreenect) subdevices that will be open by any future call to freenect_open_devices.
 */
void listSelectedSubDevices();

/**
 * Select which subdevices will be activated by future call to freenect_open_devices.
 * By default all supported subdevices will be open.
 * @param subDevs int value of freenect_device_flags representing the subdevices to select
 * @return void, MYKINECT.kinect_enabled_devices will be set accordingly.
 */
void selectSubDevices(int subDevs);

/**
 * Will tell you which subdevices is supported by libFreenect.
 */
void listSupportedSubDevices();

/**
 * List Kinect's attribute (Only serial number)
 */
void listKinectAttributes();

/**
 * Close Kinect connect.
 */
void closeKinect();

/**
 * Open Kinect subdevices listed in selected subdevices.
 * By default all supported subdevices will be open.
 */
void openKinect();

/**
 * Scan context for currently connected Kinect.
 * Currently only support one kinect at a time.
 * Plan to support multiple Kinect at some point
 */
void scan();

/**
 * Initialize libFreenect
 */
void initFreenect();

/**
 * Display console help menu.
 */
void displayHelp();

/**
 * Quit KCLI
 */
void timeToQuit();

/**
 * appendChar is called when we encounter one of the {c,d,f,s} but we do not have a % before.
 * @param output We will append char to the end of this var
 * @param c The char to append
 */
void appendChar(char **output, char c);

/**
 * appendArgChar is called when we encounter %c and also called in a loop when getting %s.
 * The difference with appendChar is that appendChar will add c to the end of output
 * appendArgChar need to add c at position - 1 because the previous char was % and was copied to output buffer.
 * @param output We will append char to the end of this var
 * @param c The char to append
 */
void appendArgChar(char **output, char c);

/**
 * Called to append int to output
 * @param output We will append char to the end of this var
 * @param d The integer to append
 */
void appendInt(char **output, int d); //Called when managing int

/**
 * Called to append double to output
 * @param output We will append char to the end of this var
 * @param f The double to append
 */
void appendDouble(char **output, double f); //Take a guess

/**
   there is no appendString... this section is managed withing pushToOutBuffer, we use appendArgChar and appendChar to manage %s
*/

/**
 * This is the function that will be called to add an output message to outputBuffer.
 * It takes the same format as printf (%c, %d, %s, %f)
 * It passes through each element and call appropriate function.
 * Once completed it shifts each element in console.outBuf and add the new one at the end.
 *
 * @param *M Message to push to console.outBuf
 * @param ... Variable param to hold the value of any given %{c,d,s,f} in given *M
 */
void pushToOutBuffer (char *M, ...);

/**
 * Called when ENTER is pressed and will processed the value inside console.Buf
 */
void processCmd();
/**
 * Trigger Kinect rgb/depth feed on/off.
 * @param FEED
 * @return 0 on success, 1 on error
 */
int triggerFeed(FEED f);

/**
 * Initialize instance of console and set default value.
 * Is automatically called.
 */
void initConsole();

/**
 * Update console information on user screen
 */
void updateConsole();

/**
 * Called by updateConsole to render a string in our console
 */
void renderString(float x, float y, int r, int g, int b, void *font, const char* string);

/**
 * Called by updateConsole to render integer in our console.
 */
void renderInt(float x, float y, int r, int g, int b, void *font, int val);

/**
 * TODO
 */
void DrawGLScene();

/**
 * TODO
 */
void ReSizeGLScene(int Width, int Height);

/**
 * TODO
 */
void InitGL(int Width, int Height);

/**
 * TODO
 */
void *gl_threadfunc(void *arg);

/**
 * TODO
 */
void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp);

/**
 * TODO
 */
void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp);

/**
 * TODO
 */
void *freenect_threadfunc(void *arg);
