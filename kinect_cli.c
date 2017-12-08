/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 * 
 * This file has been additionally modified by: Tim Flaman
 */

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

#define CONSOLE_MAX_ROWS 10
#define CONSOLE_ROWS_HEIGHT 12.0
#define MAX_OUT_BUFFER_ROWS 9

const char *commandsList[] = { "set", "trigger", "quit", "help"};
const char *commandsHelp[]  = { "Set properties.", "Trigger feeds on/off.", "Exit KinectCLI.", "Display this message."};



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

void renderString(float x, float y, int r, int g, int b, void *font, const char* string);
void renderInt(float x, float y, int r, int g, int b, void *font, int val);
void updateConsole();
void initConsole(float screenh);
void processCmd();
int triggerFeed(FEED f);

// Function definition for OutBuf management.

/*
  appendChar is called when we encounter one of the {c,d,f,s} but we do not have a % before.
 */
void appendChar(char **output, char c);

/*
  appendArgChar is called when we encounter %c and also called in a loop when getting %s.
  The difference with appendChar is that appendChar will add c to the end of output
  appendArgChar need to add c at position - 1 because the previous char was % and was copied to output buffer.
 */
void appendArgChar(char **output, char c);

void appendInt(char **output, int d); //Called when managing int
void appendDouble(char **output, double f); //Take a guess

// there is no appendString... this section is managed withing pushToOutBuffer, we use appendArgChar and appendChar to manage %s

/*
  This is the function that will be called to add an output message to outputBuffer.
  It takes the same format as printf (%c, %d, %s, %f)
  It passes through each element and call appropriate function.
  Once completed it shifts each element in outputBuffer and add the new one at the end.
 */
void pushToOutBuffer (char *M, ...);

void timeToQuit();
void displayHelp();

int depth;
char *display_name;

Display *display;
Window main_window;
Window root_window;

pthread_t freenect_thread;
volatile int die = 0;

int g_argc;
char **g_argv;

int window;

float tmprot = 1;

pthread_mutex_t gl_backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;

uint8_t gl_depth_front[640*480*4];
uint8_t gl_depth_back[640*480*4];

uint8_t gl_rgb_front[640*480*4];
uint8_t gl_rgb_back[640*480*4];

GLuint gl_depth_tex;
GLuint gl_rgb_tex;

freenect_context *f_ctx;
freenect_device *f_dev;

int freenect_angle;
int freenect_led;


float pointerx = 0, pointery = 0;
float mousex = 0, mousey = 0;
float tmousex = 0, tmousey = 0;
int screenw = 0, screenh = 0;
int snstvty;

int pause = 0;
int pusx = 0, pusy = 0; 


pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;
int got_frames = 0;

uint16_t t_gamma[2048];

console con;

// This need to be better understood and fixed.
int DrawGLSceneY = 480;
int ReSizeGLSceneY = 480;
int gl_threadfunY1 = 480;
int gl_threadfunY2 = 480;

void displayHelp(){
  size_t i = 0;
  for ( i = 0; i < sizeof(commandsList) / sizeof(commandsList[0]); i++){
    pushToOutBuffer("%s : %s", commandsList[i], commandsHelp[i]);
  }
}

void timeToQuit(){
  pushToOutBuffer ("Time to quit. Have a good night.");
  debug ("Time to quit.");
  // This hangs if no stream are open, for now, lets just make sure we have 1 open.
  if (con.Depth == 1 && con.Rgb == 1)
    triggerFeed(RGB); //RGB is faster to trigger then depth
  die = 1;

  pthread_join(freenect_thread, NULL);
  glutDestroyWindow(window);
  pthread_exit(NULL);

}

void appendChar (char **output, char c){
  char *tmpOutput = NULL;
  int outputLen = 0, tmpOutputLen = 0;

  if (*output != NULL) outputLen = strlen(*output);
  if (tmpOutput != NULL) tmpOutputLen = strlen(tmpOutput);

  tmpOutput = malloc(outputLen + 2) ; //1 for char, 1 for null char
  check_mem(tmpOutput); //From debug macro, test if malloc was successful if not go to :error
  memcpy(tmpOutput, *output, outputLen);


  tmpOutput[outputLen] = c;
  tmpOutput[outputLen + 1] = '\0';
  tmpOutputLen = strlen(tmpOutput);

  if (*output != NULL) free (*output);
  *output = malloc(tmpOutputLen + 1);
  check_mem(*output);
  memcpy(*output,tmpOutput, tmpOutputLen + 1); //+1 includes null char at the end

  if (tmpOutput != NULL) free (tmpOutput);
  return;

 error:
  if (tmpOutput != NULL) free (tmpOutput);
  // This would really only be reached in case of malloc error.
  pushToOutBuffer ("An error has happened in appendChar");
  return;
}

void appendArgChar(char **output, char c){
  char *tmpOutput = NULL;
  int outputLen = 0, tmpOutputLen = 0;

  if (*output != NULL) outputLen = strlen(*output);
  if (tmpOutput != NULL) tmpOutputLen = strlen(tmpOutput);

  tmpOutput = malloc(outputLen + 1);
  check_mem(tmpOutput);
  memcpy(tmpOutput, *output, outputLen);

  tmpOutput[outputLen - 1] = c;
  tmpOutput[outputLen] = '\0';
  tmpOutputLen = strlen(tmpOutput);

  if (*output != NULL) free (*output);
  *output = malloc(tmpOutputLen);
  check_mem(*output);
  memcpy(*output,tmpOutput, tmpOutputLen + 1);

  if (tmpOutput != NULL) free (tmpOutput);
  return;

 error:
  if (tmpOutput != NULL) free (tmpOutput);
  pushToOutBuffer ("An error has happened in appendArgChar");
  return;
}

void appendInt(char **output, int d){
  char *tmpOutput = NULL, *cs = NULL;
  int outputLen = 0, tmpOutputLen = 0, csLen = 0, malSize = 0;

  if (*output != NULL) outputLen = strlen(*output);

  /*
    Without using malloc for cs, I loose the value after memcpy.
    Both cs and tmpOutput end up sharing memory space without malloc and cause conflict. 
    At least that is what I understood as cs would become the value cs would alwaays become the same as output
    I also tried for testing to reassign cs after first memcpy using snprintf again, and then output had the same value as cs
  */
  cs = malloc(sizeof(char) * 1024);
  snprintf(cs, 1024, "%d", d);
  csLen = strlen(cs);
  malSize = outputLen + csLen;

  tmpOutput = malloc(malSize);
  check_mem(tmpOutput);
  memcpy(tmpOutput, *output, outputLen);

  int i = 0;
  for (i = 0; i < csLen; i++){
    tmpOutput[outputLen + i - 1] = cs[i];
  }
  tmpOutput[outputLen + csLen - 1] = '\0';
  tmpOutputLen = strlen(tmpOutput);

  if (*output != NULL) free (*output);
  *output = malloc(tmpOutputLen);
  check_mem(*output);
  memcpy(*output,tmpOutput, tmpOutputLen);

  if (tmpOutput != NULL) free (tmpOutput);
  if (cs != NULL) free (cs);
  return;

 error:
  pushToOutBuffer ("An error happened in appendInt");
  if (tmpOutput != NULL) free (tmpOutput);
  if (cs != NULL) free (cs);
  return;
}

void appendDouble(char **output, double f){
  char *tmpOutput = NULL, *cs = NULL;
  int outputLen = 0, tmpOutputLen = 0, csLen = 0, malSize = 0;

  if (*output != NULL) outputLen = strlen(*output);

  cs = malloc(sizeof(char) * 1024);
  snprintf(cs, 1024, "%f", f);
  csLen = strlen(cs);
  malSize = outputLen + csLen + 1;

  tmpOutput = malloc(malSize);
  check_mem(tmpOutput);
  memcpy(tmpOutput, *output, outputLen);

  int i = 0;
  for (i = 0; i < csLen; i++){
    tmpOutput[outputLen + i - 1] = cs[i];
  }

  tmpOutput[outputLen + csLen] = '\0';
  tmpOutputLen = strlen(tmpOutput);

  if (*output != NULL) free (*output);
  *output = malloc(tmpOutputLen + 1);
  check_mem(*output);
  memcpy(*output,tmpOutput, tmpOutputLen);

  if (tmpOutput != NULL) free (tmpOutput);
  if (cs != NULL) free (cs);
  return;

 error:
  pushToOutBuffer ("An error happened in appendInt");
  if (tmpOutput != NULL) free (tmpOutput);
  if (cs != NULL) free (cs);
  return;

}

void pushToOutBuffer (char *M, ...){
  va_list ap;
  int d;
  char c, *s;
  double f;
  char *output = NULL;
  va_start(ap, M);

  while (*M){
    switch (*M){
    case 'd':
      if (*--M == 37){ // If previous char == %
        *++M;
        d = va_arg(ap, int);
        appendInt(&output, d);
      }
      else{ // Just copy the char
        *++M;
        appendChar(&output, *M);
      }
      break;

    case 'c':
      if (*--M == 37){
        *++M;
        c = (char) va_arg(ap, int);
        appendArgChar(&output, c);
      }
      else{
        *++M;
        appendChar(&output, *M);
      }
      break;

    case 'f':
      if (*--M == 37){
        *++M;
        f = va_arg(ap, double);
        appendDouble(&output, f);
      }
      else{
        *++M;
        appendChar(&output, *M);
      }
      break;

    case 's':
      if (*--M == 37){
        *++M;
        s = va_arg(ap, char*);
        int i, sLen;
        sLen = strlen(s);
        appendArgChar(&output, s[0]);
        for (i = 1; i < sLen; i++) appendChar (&output, s[i]);
      }
      else{
        *++M;
        appendChar(&output, *M);
      }
      break;

    default:
      appendChar(&output, *M);
      break;
    }

    *++M;
  }
  va_end(ap);

  //Push output to outBuffer, this should act like a shift of the array.
  int k;
  for (k = 0; k < MAX_OUT_BUFFER_ROWS - 1; k++){
    con.OutBuf[k] = con.OutBuf[k+1];
  }
  //Push in new output.
  con.OutBuf[MAX_OUT_BUFFER_ROWS - 1] = output;
  return;

 error:
  debug ("An error has happened in pushToOutBuffer, I just don't want it to call itself....");
  va_end(ap);
  if (output != NULL) free (output);
  return;
}

void processCmd(){
  debug("Processing command: %s", con.Buf);
  char *sections[25];
  char *token;
  int i = 0;
  while ( (token = strsep(&con.Buf, " ")) != NULL){
    check (i < 25, "Error too many arguments."); //This will never fire with bug 24
    sections[i++] = token;
  }

  free(con.Buf);
  con.Buf = NULL;
  con.len = 0;
  if (strcmp(sections[0], "set") == 0){

    if (strcmp(sections[1],"angle") == 0){
      int angle = atoi(sections[2]);
      if (angle >= 29){
        pushToOutBuffer ("Maximum angle 28.");
        return;
      }
      if (angle <= -29){
        pushToOutBuffer ("Minimum angle -28.");
        return;
      }
      con.Angle = angle;
      freenect_angle = con.Angle;
      freenect_set_tilt_degs(f_dev,freenect_angle);
      pushToOutBuffer ("Setting angle to %d", angle);
    }


    else if (strcmp(sections[1], "led") == 0){
      // OFF
      if (strcmp(sections[2], "off") == 0){
        check (freenect_set_led(f_dev,LED_OFF) == 0, "Error setting LED to off.");
        pushToOutBuffer("LED is now OFF.");
        con.LED = LED_OFF;
      }

      //GREEN
      else if (strcmp(sections[2], "green") == 0){
        check (freenect_set_led(f_dev,LED_GREEN) == 0, "Error setting LED to green.");
        pushToOutBuffer("LED is now Green.");
        con.LED = LED_GREEN;
      }

      //Red
      else if (strcmp(sections[2], "red") == 0){
        check (freenect_set_led(f_dev,LED_RED) == 0, "Error setting LED to red.");
        pushToOutBuffer("LED is now Red.");
        con.LED = LED_RED;
      }

      //Yellow
      else if (strcmp(sections[2], "yellow") == 0){
        check(freenect_set_led(f_dev,LED_YELLOW) == 0, "Error setting LED to yellow.");
        pushToOutBuffer("LED is now Yellow.");
        con.LED = LED_YELLOW;
      }

      //blink
      else if (strcmp(sections[2], "blink") == 0){
        //green
        if (strcmp(sections[3], "green") == 0){
          check (freenect_set_led(f_dev,LED_BLINK_GREEN) == 0, "Error setting LED to blinking green.");
          pushToOutBuffer("LED is now blinking Green.");
          con.LED = LED_BLINK_GREEN;
        }
        //Red-Yellow
        else if (strcmp(sections[3], "red") == 0){
          check (freenect_set_led(f_dev,LED_BLINK_RED_YELLOW) == 0, "Error setting LED to blinking red and yellow.");
          pushToOutBuffer("LED is now blinking Red and Yellow.");
          con.LED = LED_BLINK_RED_YELLOW;
        }
        else{
          pushToOutBuffer ("Invalid blink option: green, red.");
        }
      }
      else{
        pushToOutBuffer ("Invalid LED color: off, green, red, yellow, blink green, blink red.");
      }
    }

    else if (strcmp(sections[1], "log") == 0){
      //LEVEL
      //Can we do a check on this?
      if (strcmp(sections[2], "level") == 0){
        //FATAL
        if (strcmp(sections[3], "fatal") == 0) freenect_set_log_level(f_ctx, FREENECT_LOG_FATAL);
        //ERROR
        else if (strcmp(sections[3], "error") == 0) freenect_set_log_level(f_ctx, FREENECT_LOG_ERROR);
        //WARNING
        else if (strcmp(sections[3], "warning") == 0) freenect_set_log_level(f_ctx, FREENECT_LOG_WARNING);
        //NOTICE
        else if (strcmp(sections[3], "notice") == 0) freenect_set_log_level(f_ctx, FREENECT_LOG_NOTICE);
        //INFO
        else if (strcmp(sections[3], "info") == 0) freenect_set_log_level(f_ctx, FREENECT_LOG_INFO);
        //DEBUG
        else if (strcmp(sections[3], "debug") == 0) freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
        //SPEW
        else if (strcmp(sections[3], "spew") == 0) freenect_set_log_level(f_ctx, FREENECT_LOG_SPEW);
        //FLOOD
        else if (strcmp(sections[3], "flood") == 0) freenect_set_log_level(f_ctx, FREENECT_LOG_FLOOD);
        //UNKNOWN
        else 
          pushToOutBuffer ("Unknown log level, please select between fatal, error, warning, notice, info, debug, spew, flood"); // this should not be hardcoded, can we derive from libfreenect information
      }

    }

    else {
      pushToOutBuffer ("Invalid set command: angle <int> led <{off, green, red, yellow, blink green, blink red}>");
    }
  }

  else if (strcmp(sections[0], "trigger") == 0){
    if (strcmp(sections[1], "depth") == 0){
      triggerFeed(DEPTH);
    }

    else if (strcmp(sections[1], "rgb") == 0){
      triggerFeed(RGB);
    }

    else{
      pushToOutBuffer ("Invalid trigger option: depth, rgb.");
    }
  }

  else if (strcmp(sections[0], "quit") == 0){
    timeToQuit();
  }

  else if (strcmp(sections[0], "help") == 0){
    displayHelp();
  }

  else {
    pushToOutBuffer ("Invalid command: set, trigger");
  }

  return;
 error:
  pushToOutBuffer(USER_ERR_MSG);
  free(con.Buf);
  free(USER_ERR_MSG);
}

int triggerFeed (FEED f){
  debug ("Triggering feed");
  switch (f){
  case DEPTH:
    debug ("Target feed: Depth.");
    if (con.Depth == 1){
      debug ("Starting Depth feed.");
      check (freenect_start_depth(f_dev) == 0, "Error starting depth stream");
      con.Depth = 0;
      debug ("Depth feed started.");
      pushToOutBuffer ("Depth feed started");
    }
    else{
      debug ("Stopping Depth feed.");
      check (freenect_stop_depth(f_dev) == 0, "Error stopping depth stream.");
      con.Depth = 1;
      debug ("Depth feed stopped.");
      pushToOutBuffer ("Depth feed stopped");
    }
    debug ("Depth trigger, new frame point, X1: %d, X2: %d.", con.DepthFrameX1, con.DepthFrameX2);
    break;

  case RGB:
    debug ("Target feed: RGB.");
    if (con.Rgb == 1){
      debug ("Starting RGB feed.");
      check (freenect_start_video(f_dev) == 0, "Error starting RGB stream.");
      con.Rgb = 0;
      debug ("RGB feed started");
      pushToOutBuffer ("RGB feed started.");
    }
    else{
      debug ("Stopping Rgb feed.");
      check (freenect_stop_video(f_dev) == 0, "Error stopping RGB stream");
      con.Rgb = 1;
      debug ("RGB feed stopped.");
      pushToOutBuffer ("RGB feed stopped.");
    }
    debug ("Rgb trigger, new frame point, X1: %d, X2: %d.", con.RgbFrameX1, con.RgbFrameX2);
    break;
  }
  return 0;

 error:
  pushToOutBuffer (USER_ERR_MSG);
  free (USER_ERR_MSG);
}

void initConsole(float screenh){
  debug ("Setting console default value");
  con.Sensitivity = 2000;
  con.Rgb = 1;
  con.Depth = 1;
  con.LED = LED_GREEN;
  con.Angle = 0;
  con.Buf = NULL;
  con.DepthFrameX1 = 0;
  con.DepthFrameX2 = 640;
  con.DepthFrameY1 = 200;
  con.DepthFrameY2 = 478;

  con.RgbFrameX1 = 640;
  con.RgbFrameX2 = 1280;
  con.RgbFrameY1 =  200;
  con.RgbFrameY2 = 478;
  int i;
  for (i = 0; i < CONSOLE_MAX_ROWS; i++){
      debug ("Row %d = %f", i, (i == 0 ? 1 : con.Rows[i - 1] + CONSOLE_ROWS_HEIGHT));
      con.Rows[i] = (i == 10 ? 1 : con.Rows[i - 1] + CONSOLE_ROWS_HEIGHT);
  }
  debug ("Done intializing console.");
  pushToOutBuffer ("Console is ready.");
}

void updateConsole(){

  // Status bar on the left
  renderString (150.0, con.Rows[0], ( con.Rgb == 1 ? 200 : 0 ), ( con.Rgb == 0 ? 200 : 0 ), 0, GLUT_BITMAP_HELVETICA_12, "RGB");

  renderString (100.0, con.Rows[0],  ( con.Depth == 1 ? 200 : 0 ), ( con.Depth == 0 ? 200 : 0 ), 0, GLUT_BITMAP_HELVETICA_12, "DEPTH");

  renderString (150.0, con.Rows[1], 200, 0, 0, GLUT_BITMAP_HELVETICA_12, "LED: ");
  renderInt (100.0, con.Rows[1], 0, 200, 0, GLUT_BITMAP_HELVETICA_12, con.LED);

  renderString (150.0, con.Rows[2], 200, 0, 0, GLUT_BITMAP_HELVETICA_12, "Angle: ");
  renderInt (100.0, con.Rows[2], 0, 200, 0, GLUT_BITMAP_HELVETICA_12, con.Angle);

  // INPUT
  renderString (1270.0, con.Rows[CONSOLE_MAX_ROWS - 1], 0, 200, 0, GLUT_BITMAP_HELVETICA_12, con.Buf);

  //OUTPUT
  int i;
  for (i = 0; i < CONSOLE_MAX_ROWS - 1; i++){
    renderString (1270.0, con.Rows[i], 200, 0, 0, GLUT_BITMAP_HELVETICA_12, con.OutBuf[i]);
  }
}

void DrawGLScene()
{
  if (con.Rgb == 0 || con.Depth == 0){
    pthread_mutex_lock(&gl_backbuf_mutex);

    while (got_frames < 2) {
      pthread_cond_wait(&gl_frame_cond, &gl_backbuf_mutex);
    }


    if (con.Depth == 0)
      memcpy(gl_depth_front, gl_depth_back, sizeof(gl_depth_back));
    if (con.Rgb == 0)
      memcpy(gl_rgb_front, gl_rgb_back, sizeof(gl_rgb_back));
    got_frames = 0;
    pthread_mutex_unlock(&gl_backbuf_mutex);
  }

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);
  glTranslated(1280, 0, 0);
  glScalef(-1, 1, 1);


  if (con.Depth == 0){
    glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, DrawGLSceneY, 0, GL_RGB, GL_UNSIGNED_BYTE, gl_depth_front);

    glBegin(GL_TRIANGLE_FAN);
    glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
    glTexCoord2f(0, 0); glVertex3f(con.DepthFrameX1,con.DepthFrameY1,0);
    glTexCoord2f(1, 0); glVertex3f(con.DepthFrameX2,con.DepthFrameY1,0);
    glTexCoord2f(1, 1); glVertex3f(con.DepthFrameX2,con.DepthFrameY2,0);
    glTexCoord2f(0, 1); glVertex3f(con.DepthFrameX1,con.DepthFrameY2,0);
    glEnd();
  }

  if (con.Rgb == 0){
    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, DrawGLSceneY, 0, GL_RGB, GL_UNSIGNED_BYTE, gl_rgb_front);

    glBegin(GL_TRIANGLE_FAN);
    glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
    glTexCoord2f(0, 0); glVertex3f(con.RgbFrameX1,con.RgbFrameY1,0);
    glTexCoord2f(1, 0); glVertex3f(con.RgbFrameX2,con.RgbFrameY1,0);
    glTexCoord2f(1, 1); glVertex3f(con.RgbFrameX2,con.RgbFrameY2,0);
    glTexCoord2f(0, 1); glVertex3f(con.RgbFrameX1,con.RgbFrameY2,0);

    glEnd();

  }
  // Disable texture to write text in console.
  glDisable(GL_TEXTURE_2D);
  updateConsole();
  glutSwapBuffers();

}

void renderString(float x, float y, int r, int g, int b, void *font, const char* string){
  glColor3f(r, g, b);
  glRasterPos2f(x, y);
  glutBitmapString(font, string);
}

void renderInt(float x, float y, int r, int g, int b, void *font, int val){
  char buf[sizeof(int)*3+2];
  snprintf(buf, sizeof(buf), "%d", val);
  glColor3f(r, g, b);
  glRasterPos2f(x, y);
  glutBitmapString(font, buf);
}

void keyPressed(unsigned char key, int x, int y)
{
  char *tmpBuf = NULL;

  switch(key) {
  case 27: //ESC
    debug ("Quitting on ESC key.");
    timeToQuit();
    break;

  case 8: //backspace
    if (con.Buf == NULL) break;
    con.len = strlen(con.Buf);
    if (con.len == 0) break;

    if (con.len == 1){
      con.Buf = NULL;
      con.len = 0;
      break;
    }

    if (tmpBuf != NULL) free(tmpBuf);
    tmpBuf = malloc(con.len - 1);
    check_mem(tmpBuf);

    int i;
    for (i = 0; i < con.len - 1; i++){
      tmpBuf[i] = con.Buf[i];
    }
    tmpBuf[con.len - 1] = '\0';

    free (con.Buf);
    con.Buf = (char *) malloc(sizeof(tmpBuf));
    check_mem(con.Buf);
    strcpy(con.Buf, tmpBuf);

    free(tmpBuf);
    break;

  case 13: //ENTER
    if (con.Buf == NULL){
      debug ("con.Buf is null.");
      break;
    }
    con.len = strlen(con.Buf);
    if (con.len == 0){
      debug ("con.len == 0.");
      break;
    }
    else {
      processCmd();
    }
    break;

  default: // Append char to con.conInput
    if (con.Buf == NULL)
      con.len = 0;

    else
      con.len = strlen(con.Buf);

    /* 
       There is a bug when con.len will reach 25, we will get a double free or memory corruption, 
       either (fast) if you type fast of (!prev) if you type slow.
       I have not been able to trace the corruption yet.
    */
    if (con.len == 23){
      debug ("BUG 24 Patch activated....");
      pushToOutBuffer ("CLI character limit reached. Yes, this is a bug.");
      return;
    }
    if (tmpBuf != NULL) free (tmpBuf);
    tmpBuf = malloc(con.len + 1 + 1); //one for extra char and 1 for trailling zero
    check_mem(tmpBuf);

    if (con.len > 0)
      strcpy(tmpBuf, con.Buf);
    tmpBuf[con.len] = key;
    tmpBuf[con.len + 1] = '\0';
    free (con.Buf);
    con.Buf = (char *) malloc(sizeof(tmpBuf));
    check_mem(con.Buf);
    strcpy (con.Buf, tmpBuf);

    free (tmpBuf);
    break;

  }
  return;
 error:
  debug ("Error with malloc in processCmd().");
  pushToOutBuffer ("Error with malloc in processCmd(). We might crash.");
  debug ("Attempting to free tmpBuf to prevent memory leaks, this may crash.");
  free (tmpBuf);
  pushToOutBuffer ("Looks like we did not.");
}

void ReSizeGLScene(int Width, int Height)
{
	glViewport(0,0,Width,Height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (0, 1280, ReSizeGLSceneY, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
}

void InitGL(int Width, int Height)
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glDepthFunc(GL_LESS);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_SMOOTH);
	glGenTextures(1, &gl_depth_tex);
	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glGenTextures(1, &gl_rgb_tex);
	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	ReSizeGLScene(Width, Height);
}

void *gl_threadfunc(void *arg)
{
	debug("GL thread\n");

	glutInit(&g_argc, g_argv);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(1280, gl_threadfunY1);
	glutInitWindowPosition(0, 0);

	window = glutCreateWindow("Kinect Control");
	glutDisplayFunc(&DrawGLScene);
	glutIdleFunc(&DrawGLScene);
	glutReshapeFunc(&ReSizeGLScene);
	glutKeyboardFunc(&keyPressed);

	InitGL(1280, gl_threadfunY2);
 
	glutMainLoop();

	return NULL;
}

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	int i;
	int first = 0;
	int px = 0 , py = 0;
	int tx = 0 , ty = 0;
	int alert = 0;
	uint16_t *depth = v_depth;
	pthread_mutex_lock(&gl_backbuf_mutex);
	for (i=0; i<FREENECT_IR_FRAME_PIX; i++) {
		int pval = t_gamma[depth[i]];
		int lb = pval & 0xff;

		tx++;
		if(tx >= 640) {
			tx = 0;
			ty++;
		}
		/*case 0-5*/
		switch (pval>>8) {
			case 0:
				gl_depth_back[3*i+0] = 255;
				gl_depth_back[3*i+1] = 0;
				gl_depth_back[3*i+2] = 0;
				alert++;
				if (!first) { 
					first = i;
					px = tx;
					py = ty;
				}
				break;
			case 1:
				gl_depth_back[3*i+0] = 255;
				gl_depth_back[3*i+1] = 255;
				gl_depth_back[3*i+2] = 255;
				break;
			default:
				gl_depth_back[3*i+0] = 0;
				gl_depth_back[3*i+1] = 0;
				gl_depth_back[3*i+2] = 0;
				break;
		}
	}

  /*
	if(alert > snstvty) {	
		debug("!!!TOO CLOSE!!!");
	} else {
    if(first) {
			pointerx = ((px-640.0f) / -1);
			pointery = (py);
			mousex = ((pointerx / 630.0f) * screenw);
			mousey = ((pointery / 470.0f) * screenh);
			int mx , my;
			mx = mousex;
			my = mousey;

			if(mx > tmousex) tmousex+= (mx - tmousex) / 7;
			if(mx < tmousex) tmousex-= (tmousex - mx) / 7;
			if(my > tmousey) tmousey+= (my - tmousey) / 7;
			if(my < tmousey) tmousey-= (tmousey - my) / 7;			

			if((pusx <= (mx + 15))  && (pusx >= (mx - 15)) && (pusy <= (my + 15))  && (pusy >= (my - 15))) {
				pause++;
 				debug("%d", pause);
			} else {
				pusx = mx;
				pusy = my;
				pause = 0;
			}		

			if(pause > 15) {
				pause = -30;
				XTestFakeButtonEvent(display, 1, TRUE, CurrentTime);
				XTestFakeButtonEvent(display, 1, FALSE, CurrentTime);
			}

			//printf("-- %d x %d -- \n", mx, my);

			XTestFakeMotionEvent(display, -1, tmousex-200, tmousey-200, CurrentTime);
			XSync(display, 0);

			//printf("\n\n %d  -  %d \n\n", mx, my);


		}
    }*/


	got_frames++;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	pthread_mutex_lock(&gl_backbuf_mutex);
	got_frames++;
	memcpy(gl_rgb_back, rgb, FREENECT_VIDEO_RGB_SIZE);
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);

}

void *freenect_threadfunc(void *arg)
{
  debug ("Init freenect thread function.");
  freenect_video_format fvf;
	freenect_set_tilt_degs(f_dev,freenect_angle);
	check (freenect_set_led(f_dev,LED_GREEN) == 0, "Error setting led to green.");
	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_video_callback(f_dev, rgb_cb);
  debug("Video mode count: %d", freenect_get_video_mode_count());
  debug("Commented out set video mode, I had bad results, need more research");
  /*
    freenect_frame_mode myMode = freenect_get_video_mode(FREENECT_VIDEO_RGB);
    freenect_set_video_mode(f_dev, myMode);
    freenect_set_depth_format(f_dev, FREENECT_DEPTH_11BIT);
  */

  debug ("Entering freenect main loop.");
	while(!die && freenect_process_events(f_ctx) >= 0 )
	{
    //		freenect_raw_tilt_state* state;
		freenect_update_tilt_state(f_dev);
    //		state = freenect_get_tilt_state(f_dev);;
		//double dx,dy,dz;
		//freenect_get_mks_accel(state, &dx, &dy, &dz);
		//fflush(stdout);
	}

	debug("Shutting Down Streams...");
  if (con.Depth == 0){
    debug("Stopping depth stream.");
    freenect_stop_depth(f_dev);
  }
  if (con.Rgb == 0){
    debug("Stopping rgb stream.");
    freenect_stop_video(f_dev);
  }

  debug("Closing device.");
	freenect_close_device(f_dev);

  debug("Shutting it down.");
	freenect_shutdown(f_ctx);

  debug("Free command buffer");
  free(con.Buf);

  debug("-- done!");

	return NULL;

 error:
  debug ("Error in freenect_threadfunc.");
}

int main(int argc, char **argv)
{
  debug("Let's get started");
	int res;

	display = XOpenDisplay(0);

	root_window = DefaultRootWindow(display);

	screenw = XDisplayWidth(display, SCREEN);
	screenh = XDisplayHeight(display, SCREEN);

	debug("Default Display Found.");
	debug("Size: %dx%d.", screenw, screenh);

  // What is this?
	screenw += 200;
	screenh += 200;

	int i;
	for (i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}

	g_argc = argc;
	g_argv = argv;

  debug ("Init console");
  initConsole(480); //This var should not be hardcoded. Default should be given with choice of changing. I think

  debug("Initializing Kinect");
  check(freenect_init(&f_ctx, NULL) == 0," freenect_init failed");
  freenect_angle = con.Angle;
  debug("Set freenect log level %d", FREENECT_LOG_DEBUG);
	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);

	int nr_devices = freenect_num_devices (f_ctx);
	debug ("Number of Devices Found: %d", nr_devices);

	int user_device_number = 0;
	if (argc > 1)
		user_device_number = atoi(argv[1]);

  check (nr_devices >= 1, "nr_devices is below 1.");

  check (freenect_open_device(f_ctx, &f_dev, user_device_number) >= 0, "Could not locate Kinect");


	res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
  check (!res, "Could not create thread.");

	gl_threadfunc(NULL);


	return 0;

 error:

  return 1;
}
