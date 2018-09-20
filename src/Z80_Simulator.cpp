// Z80_Simulator.cpp : Defines the entry point for the console application.
//

// ChipBrowser.cpp : Defines the entry point for the console application.
// 8085 CPU Simulator
// Version: 1.0
// Author: Pavel Zima
// Date: 1.1.2013

// Adapted to Z80 CPU Simulator on 26.9.2013
// Ported to Linux/g++ by Dave Banks 29.8.2018

#include <inttypes.h>
#include <locale.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <png++/png.hpp>

#include <sys/time.h>

using namespace std;

// DMB: Not a problem on Linux!
// it says that fopen is unsafe
//#pragma warning(disable : 4996)

// DMB: Need to use ulimit -s 131072
// we need really big stack for recursive flood fill (need to be reimplemented)
//#pragma comment(linker, "/STACK:536870912")

#define FILL_STACK_SIZE 10000000

uint16_t stack_x[FILL_STACK_SIZE];
uint16_t stack_y[FILL_STACK_SIZE];
uint16_t stack_layer[FILL_STACK_SIZE];

#define ZeroMemory(p, sz) memset((p), 0, (sz))

uint64_t GetTickCount()
{
   timeval tv;
   gettimeofday(&tv, NULL);
   return  tv.tv_sec * 1000LL + tv.tv_usec / 1000;
}

#ifdef DMB_THREAD
unsigned int thread_count = 3;
#endif

unsigned int DIVISOR = 600; // the lower the faster is clock, 1000 is lowest value I achieved
#define MINSHAPESIZE 25 // if a shape is smaller than this it gets reported

#define VCC 5.0f
#define QUANTUM 100.0f // basic quantum of charge moved in one cycle
#define MAXQUANTUM 600.0f // charge gets truncated to this value
#define PULLUPDEFLATOR 1.1f // positive charge gets divided by this

#define METAL 1
#define POLYSILICON 2
#define DIFFUSION 4
#define VIAS 8
#define BURIED 16 // layer
#define PADS 32
#define TRANSISTORS 64
#define VIAS_TO_POLYSILICON 128
#define VIAS_TO_DIFFUSION 256
#define BURIED_CONTACT 512 // real contact
#define REAL_DIFFUSION 1024 // i.e. no transistor
#define ION_IMPLANTS 2048 // Z80 traps
#define TEMPORARY 32768

// basic signals - they get their own color while exporting figures

#define GND 1
#define SIG_VCC 2
#define PAD_CLK 3
#define SIG_RESET 4

// definition of pads

#define PAD_A0 5
#define PAD_A1 6
#define PAD_A2 7
#define PAD_A3 8
#define PAD_A4 9
#define PAD_A5 10
#define PAD_A6 11
#define PAD_A7 12
#define PAD_A8 13
#define PAD_A9 14
#define PAD_A10 15
#define PAD_A11 16
#define PAD_A12 17
#define PAD_A13 18
#define PAD_A14 19
#define PAD_A15 20

#define PAD__RESET 21
#define PAD__WAIT 22
#define PAD__INT 23
#define PAD__NMI 24
#define PAD__BUSRQ 25

#define PAD__M1 26
#define PAD__RD 27
#define PAD__WR 28
#define PAD__MREQ 29
#define PAD__IORQ 30
#define PAD__RFSH 31

#define PAD_D0 32
#define PAD_D1 33
#define PAD_D2 34
#define PAD_D3 35
#define PAD_D4 36
#define PAD_D5 37
#define PAD_D6 38
#define PAD_D7 39

#define PAD__HALT 40
#define PAD__BUSAK 41

// all other signals are numbered automaticaly

#define FIRST_SIGNAL 42

// Global variables which can be set

bool verbous = false;

int size_x, size_y;
uint16_t *pombuf;
uint16_t *signals_metal;
uint16_t *signals_poly;
uint16_t *signals_diff;

// trying to keep FillObject local heap as small as possible
int type, type2, type3;

png::image<png::rgb_pixel> bd;

int shapesize;
bool objectfound;
int nextsignal;

#define GATE 1
#define DRAIN 2
#define SOURCE 3

uint8_t memory[65536];
uint8_t ports[256];

// Connection to transistor remembers index of connected transistor and its terminal
// proportion is the proportion of that transisor are to the area of all transistors connected
// to the respective signal - it is here for optimalisation purposes

class Connection
{
public:
   Connection();
   int terminal;
   int index;
   float proportion;
};

Connection::Connection()
{
   terminal = index = 0;
   proportion = 0.0f;
}

#define SIG_GND 1
#define SIG_PWR 2
#define SIG_FLOATING 3

// Signal keeps the vector of Connections (i.e. all transistors connected to the respective signal)
// Homogenize() averages the charge proportionally by transistor area
// ignore means that this signal need not to be homogenized - a try for optimalization
// but it works only for Vcc and GND

class Signal
{
public:
   Signal();
   void Homogenize();
   vector<Connection> connections;
   float signalarea;
   bool ignore;
   bool pullup;
};

Signal::Signal()
{
   signalarea = 0.0f;
   ignore = false;
   pullup = false;
}

vector<Signal> signals;

#define PAD_INPUT 1
#define PAD_OUTPUT 2
#define PAD_BIDIRECTIONAL 3

// PADs - used for communication with the CPU see its use in simulation below
class Pad
{
public:
   Pad();
   void SetInputSignal(int signal);
   float ReadOutput();
   int ReadOutputStatus();
   int ReadInputStatus();
   int type;
   int x, y;
   int origsignal;
   vector<Connection> connections;
};

Pad::Pad()
{
   type = 0;
   x = y = 0;
   origsignal = 0;
}

vector<Pad> pads;

// transistor - keeps connections to other transistors
// Simulate() - moves charge between source and drain
class Transistor
{
public:
   Transistor();
   bool IsOn();
   int IsOnAnalog();
   void Simulate();
   void Normalize();
   int Valuate();

   int x, y;
   int gate, source, drain;
   int sourcelen, drainlen, otherlen;
   float area;
   bool depletion;

   float resist;
   float gatecharge, sourcecharge, draincharge;

   vector<Connection> gateconnections;
   vector<Connection> sourceconnections;
   vector<Connection> drainconnections;
   float gateneighborhood, sourceneighborhood, drainneighborhood;
   float chargetobeon, pomchargetogo;
};

Transistor::Transistor()
{
   x = y = 0;
   gate = source = drain = 0;
   sourcelen = drainlen = otherlen = 0;
   area = 0.0f;
   depletion = false;
   resist = 0.0f;
   gatecharge = sourcecharge = draincharge = 0.0f;
   gateneighborhood = sourceneighborhood = drainneighborhood = 0.0f;
   chargetobeon = pomchargetogo = 0.0f;
}

inline bool Transistor::IsOn()
{
   if (gatecharge > 0.0f)
      return true;
   return false;
}

int Transistor::IsOnAnalog()
{
   return int(50.0f * gatecharge / area) + 50;
}

// Gets the type of the transistor - originally for optimalization purposes now more for statistical purposes
inline int Transistor::Valuate()
{
   // 1 depletion pullup
   // 2 enhancement pullup
   // 3 direct pulldown
   // 4 other

   if (depletion)
      return 1;
   if (drain == SIG_VCC)
      return 2;
   if (source == SIG_GND)
      return 3;
   return 4;
}

vector<Transistor> transistors;

#ifdef DMB_THREAD
DWORD WINAPI ThreadSimulateTransistors(LPVOID thread_id)
{
   printf("*** Thread: %d started @%d\n", thread_id, GetTickCount());
   for (unsigned int pogo = 0; pogo < 1000; pogo++)
   for (unsigned int j = (unsigned int) thread_id; j < transistors.size(); j += thread_count)
   {
      transistors[j].Simulate();
   // printf("*** Thread: %d transistor: %04d\n", thread_id, j);
   }
   printf("*** Thread: %d finished @%d\n", thread_id, GetTickCount());
   return NULL;
}

HANDLE *threadList = NULL;
#endif

void Signal::Homogenize()
{
   float pomcharge = 0.0f;
   for (unsigned int i = 0; i < connections.size(); i++)
   {
      if (connections[i].terminal == GATE)
         pomcharge += transistors[connections[i].index].gatecharge;
      else if (connections[i].terminal == SOURCE)
         pomcharge += transistors[connections[i].index].sourcecharge;
      else if (connections[i].terminal == DRAIN)
         pomcharge += transistors[connections[i].index].draincharge;
   }

   for (unsigned int i = 0; i < connections.size(); i++)
   {
      if (connections[i].terminal == GATE)
         transistors[connections[i].index].gatecharge = pomcharge * connections[i].proportion;
      else if (connections[i].terminal == SOURCE)
         transistors[connections[i].index].sourcecharge = pomcharge * connections[i].proportion;
      else if (connections[i].terminal == DRAIN)
         transistors[connections[i].index].draincharge = pomcharge * connections[i].proportion;
   }
}

void Transistor::Simulate()
{
   if (gate == SIG_GND)
      gatecharge = 0.0f;
   else if (gate == SIG_VCC)
      gatecharge = area;

   if (depletion)
   {
      if (drain == SIG_VCC)
      {
         float chargetogo = pomchargetogo;

         chargetogo /= PULLUPDEFLATOR; // pull-ups are too strong, we need to weaken them

         for (unsigned int i = 0; i < sourceconnections.size(); i++)
         {
            if (sourceconnections[i].terminal == GATE)
               transistors[sourceconnections[i].index].gatecharge += chargetogo * sourceconnections[i].proportion;
            else if (sourceconnections[i].terminal == SOURCE)
               transistors[sourceconnections[i].index].sourcecharge += chargetogo * sourceconnections[i].proportion;
            else if (sourceconnections[i].terminal == DRAIN)
               transistors[sourceconnections[i].index].draincharge += chargetogo * sourceconnections[i].proportion;
         }
      }
      else if (source == SIG_GND)
      {
         float chargetogo = pomchargetogo;

         for (unsigned int i = 0; i < drainconnections.size(); i++)
         {
            if (drainconnections[i].terminal == GATE)
               transistors[drainconnections[i].index].gatecharge -= chargetogo * drainconnections[i].proportion;
            else if (drainconnections[i].terminal == SOURCE)
               transistors[drainconnections[i].index].sourcecharge -= chargetogo * drainconnections[i].proportion;
            else if (drainconnections[i].terminal == DRAIN)
               transistors[drainconnections[i].index].draincharge -= chargetogo * drainconnections[i].proportion;
         }
      }
      else
      {
         float pomsourcecharge = 0.0f, pomdraincharge = 0.0f;

         pomsourcecharge = sourcecharge;
         if (pomsourcecharge > 0.0f)
            pomsourcecharge /= PULLUPDEFLATOR;

         pomdraincharge = draincharge;
         if (pomdraincharge > 0.0f)
            pomdraincharge /= PULLUPDEFLATOR;

         float chargetogo = ((pomsourcecharge - pomdraincharge) / resist) / PULLUPDEFLATOR;
         float pomsign = 1.0;
         if (chargetogo < 0.0f)
         {
            pomsign = -1.0;
            chargetogo = -chargetogo;
         }
         if (chargetogo > MAXQUANTUM)
            chargetogo = MAXQUANTUM;
         chargetogo *= pomsign;

         sourcecharge -= chargetogo;
         draincharge += chargetogo;
      }
   }
   else
   {
      if (IsOn())
      {
         if (drain == SIG_VCC)
         {
            float chargetogo = pomchargetogo;
            chargetogo *= gatecharge / area;
            chargetogo /= PULLUPDEFLATOR;

            for (unsigned int i = 0; i < sourceconnections.size(); i++)
            {
               if (sourceconnections[i].terminal == GATE)
                  transistors[sourceconnections[i].index].gatecharge += chargetogo * sourceconnections[i].proportion;
               else if (sourceconnections[i].terminal == SOURCE)
                  transistors[sourceconnections[i].index].sourcecharge += chargetogo * sourceconnections[i].proportion;
               else if (sourceconnections[i].terminal == DRAIN)
                  transistors[sourceconnections[i].index].draincharge += chargetogo * sourceconnections[i].proportion;
            }
         }
         else if (source == SIG_GND)
         {
            float chargetogo = pomchargetogo;
            chargetogo *= gatecharge / area;

            for (unsigned int i = 0; i < drainconnections.size(); i++)
            {
               if (drainconnections[i].terminal == GATE)
                  transistors[drainconnections[i].index].gatecharge -= chargetogo * drainconnections[i].proportion;
               else if (drainconnections[i].terminal == SOURCE)
                  transistors[drainconnections[i].index].sourcecharge -= chargetogo * drainconnections[i].proportion;
               else if (drainconnections[i].terminal == DRAIN)
                  transistors[drainconnections[i].index].draincharge -= chargetogo * drainconnections[i].proportion;
            }
         }
         else
         {
            float pomsourcecharge = 0.0f, pomdraincharge = 0.0f;

            pomsourcecharge = sourcecharge;
            if (pomsourcecharge > 0.0f)
               pomsourcecharge /= PULLUPDEFLATOR;

            pomdraincharge = draincharge;
            if (pomdraincharge > 0.0f)
               pomdraincharge /= PULLUPDEFLATOR;

            float chargetogo = ((pomsourcecharge - pomdraincharge) / resist) / PULLUPDEFLATOR;
            float pomsign = 1.0;
            if (chargetogo < 0.0f)
            {
               pomsign = -1.0;
               chargetogo = -chargetogo;
            }
            if (chargetogo > MAXQUANTUM)
               chargetogo = MAXQUANTUM;
            chargetogo *= gatecharge / area;
            chargetogo *= pomsign;

            sourcecharge -= chargetogo;
            draincharge += chargetogo;
         }
      }
   }
}

void Transistor::Normalize()
{
   if (gatecharge < -area)
      gatecharge = -area;
   if (gatecharge > area)
      gatecharge = area;
   if (sourcecharge < -area)
      sourcecharge = -area;
   if (sourcecharge > area)
      sourcecharge = area;
   if (draincharge < -area)
      draincharge = -area;
   if (draincharge > area)
      draincharge = area;
}

int Pad::ReadInputStatus()
{
   int pomvalue = SIG_FLOATING;

   if (connections.size())
   {
      if (connections[0].terminal == GATE)
         pomvalue = transistors[connections[0].index].gate;
      else if (connections[0].terminal == SOURCE)
         pomvalue = transistors[connections[0].index].source;
      else if (connections[0].terminal == DRAIN)
         pomvalue = transistors[connections[0].index].drain;
   }

   if (pomvalue > SIG_VCC)
      pomvalue = SIG_FLOATING;

   return pomvalue;
}

void Pad::SetInputSignal(int signal)
{
   for (unsigned int i = 0; i < connections.size(); i++)
   {
      if (signal != SIG_FLOATING)
      {
         if (connections[i].terminal == GATE)
            transistors[connections[i].index].gate = signal;
         else if (connections[i].terminal == SOURCE)
            transistors[connections[i].index].source = signal;
         else if (connections[i].terminal == DRAIN)
            transistors[connections[i].index].drain = signal;
      }
      else
      {
         if (connections[i].terminal == GATE)
            transistors[connections[i].index].gate = origsignal;
         else if (connections[i].terminal == SOURCE)
            transistors[connections[i].index].source = origsignal;
         else if (connections[i].terminal == DRAIN)
            transistors[connections[i].index].drain = origsignal;
      }
   }
}

float Pad::ReadOutput()
{
   float shouldbe = 0.0f, reallywas = 0.0f;
   for (unsigned int i = 0; i < connections.size(); i++)
   {
      if (connections[i].terminal == SOURCE)
      {
         shouldbe += transistors[connections[i].index].area;
         reallywas += transistors[connections[i].index].sourcecharge;
      }
      else if (connections[i].terminal == DRAIN)
      {
         shouldbe += transistors[connections[i].index].area;
         reallywas += transistors[connections[i].index].draincharge;
      }
   }

   return reallywas / shouldbe;
}

// does not work correctly
// i.e. it cannot recoginze floating status
int Pad::ReadOutputStatus()
{
   float pom = ReadOutput();
   if (pom < -0.05f)
      return SIG_GND;
   else if (pom > 0.05f)
      return SIG_VCC;
   return SIG_FLOATING;
}

int GetPixelFromBitmapData(png::image<png::rgb_pixel>& image, int x, int y)
{
   if (x < 0)
      return 0;
   if (y < 0)
      return 0;
   if (x >= size_x)
      return 0;
   if (y >= size_y)
      return 0;

   png::rgb_pixel pixel = image[y][x];

   return pixel.red + (pixel.green << 8) + (pixel.blue << 16);
}


bool SetPixelToBitmapData(png::image<png::rgba_pixel>& image, int x, int y, int barva)
{
   if (x < 0)
      return false;
   if (y < 0)
      return false;
   if (x >= size_x)
      return false;
   if (y >= size_y)
      return false;

   if (barva)
      image[y][x] = png::rgba_pixel((barva >> 16) & 0xff, (barva >> 8) & 0xff, barva & 0xff, 0xff);
   else
      image[y][x] = png::rgba_pixel();

   return true;
}

// These functions are very poorly implemented - they use recursive flood algorithm which is very slow and time consuming
// should be reimplemented

bool FillObject(int x, int y)
{
   if (pombuf[y * size_x + x] & type || !GetPixelFromBitmapData(bd, x, y)) {
      return false;
   }
   stack_x[0] = x;
   stack_y[0] = y;
   int stack_ptr = 1;

   while (stack_ptr > 0) {

      stack_ptr--;
      x = stack_x[stack_ptr];
      y = stack_y[stack_ptr];

      pombuf[y * size_x + x] |= type;
      shapesize++;

      for (int i = 0; i < 4; i++) {
         int nx;
         int ny;
         switch (i) {
         case 0: nx = x - 1; ny = y    ; break;
         case 1: nx = x    ; ny = y - 1; break;
         case 2: nx = x + 1; ny = y    ; break;
         case 3: nx = x    ; ny = y + 1; break;
         }
         if (nx >= 0 && nx < size_x && ny >= 0 && ny < size_y) {
            if (!(pombuf[ny * size_x + nx] & type) && GetPixelFromBitmapData(bd, nx, ny)) {
               if (stack_ptr < FILL_STACK_SIZE) {
                  stack_x[stack_ptr] = nx;
                  stack_y[stack_ptr] = ny;
                  stack_ptr++;
               } else {
                  printf("Fill stack overflow in FillObject()\n");
                  ::exit(1);
               }
            }
         }
      }
   }
   return true;
}


bool FillStructure(int x, int y)
{
   if ((pombuf[y * size_x + x] & type) || (pombuf[y * size_x + x] & type2) != type3) {
      return false;
   }
   stack_x[0] = x;
   stack_y[0] = y;
   int stack_ptr = 1;

   while (stack_ptr > 0) {

      stack_ptr--;
      x = stack_x[stack_ptr];
      y = stack_y[stack_ptr];

      pombuf[y * size_x + x] |= type;
      shapesize++;

      for (int i = 0; i < 4; i++) {
         int nx;
         int ny;
         switch (i) {
         case 0: nx = x - 1; ny = y    ; break;
         case 1: nx = x    ; ny = y - 1; break;
         case 2: nx = x + 1; ny = y    ; break;
         case 3: nx = x    ; ny = y + 1; break;
         }
         if (nx >= 0 && nx < size_x && ny >= 0 && ny < size_y) {
            if (!(pombuf[ny * size_x + nx] & type) && (pombuf[ny * size_x + nx] & type2) == type3) {
               if (stack_ptr < FILL_STACK_SIZE) {
                  stack_x[stack_ptr] = nx;
                  stack_y[stack_ptr] = ny;
                  stack_ptr++;
               } else {
                  printf("Fill stack overflow in FillStructure()\n");
                  ::exit(1);
               }
            }
         }
      }
   }
   return true;
}


bool FillStructureCheck(int x, int y)
{
   if ((pombuf[y * size_x + x] & type) || (pombuf[y * size_x + x] & type2) != type2) {
      return false;
   }
   stack_x[0] = x;
   stack_y[0] = y;
   int stack_ptr = 1;

   while (stack_ptr > 0) {

      stack_ptr--;
      x = stack_x[stack_ptr];
      y = stack_y[stack_ptr];

      pombuf[y * size_x + x] |= type;

      if ((pombuf[y * size_x + x] & type3)) {
         objectfound = true;
      }

      for (int i = 0; i < 4; i++) {
         int nx;
         int ny;
         switch (i) {
         case 0: nx = x - 1; ny = y    ; break;
         case 1: nx = x    ; ny = y - 1; break;
         case 2: nx = x + 1; ny = y    ; break;
         case 3: nx = x    ; ny = y + 1; break;
         }
         if (nx >= 0 && nx < size_x && ny >= 0 && ny < size_y) {
            if (!(pombuf[ny * size_x + nx] & type) && (pombuf[ny * size_x + nx] & type2) == type2) {
               if (stack_ptr < FILL_STACK_SIZE) {
                  stack_x[stack_ptr] = nx;
                  stack_y[stack_ptr] = ny;
                  stack_ptr++;
               } else {
                  printf("Fill stack overflow in FillCheckStructure()\n");
                  ::exit(1);
               }
            }
         }
      }
   }
   return true;
}

void ClearTemporary()
{
   for (int y = 0; y < size_y; y++)
      for (int x = 0; x < size_x; x++)
         pombuf[y * size_x + x] &= ~TEMPORARY;
}

// opens the respective file and copies its content to pombuff which holds the layers
void CheckFile(char *firstpart, int ltype)
{
   type = ltype;

   char filename[256];
   if (type == VIAS)
      sprintf(filename, "%s%s", firstpart, "_vias.png");
   if (type == METAL)
      sprintf(filename, "%s%s", firstpart, "_metal.png");
   if (type == PADS)
      sprintf(filename, "%s%s", firstpart, "_pads.png");
   if (type == POLYSILICON)
      sprintf(filename, "%s%s", firstpart, "_polysilicon.png");
   if (type == DIFFUSION)
      sprintf(filename, "%s%s", firstpart, "_diffusion.png");
   if (type == BURIED)
      sprintf(filename, "%s%s", firstpart, "_buried.png");
   if (type == ION_IMPLANTS)
      sprintf(filename, "%s%s", firstpart, "_ions.png");
   if (verbous)
      printf("%s", filename);

   try {

      png::image<png::rgb_pixel> bitmapa(filename);

      size_x = bitmapa.get_width();
      size_y = bitmapa.get_height();

      if (verbous)
         printf(" size x: %d, y: %d\n", size_x, size_y);

      if (!pombuf)
      {
         pombuf = new uint16_t[size_x * size_y];
         ZeroMemory(&pombuf[0], size_x * size_y * sizeof(uint16_t));
      }

      // Assign global variable, used by FillObject
      bd = bitmapa;

      int bitmap_room = 0;
      for (int y = 0; y < size_y; y++)
         for (int x = 0; x < size_x; x++)
            if (GetPixelFromBitmapData(bitmapa, x, y))
               bitmap_room++;
      if (verbous)
         printf("Percentage: %.2f%%\n", 100.0 * bitmap_room / size_x / size_y);

      int bitmap_count = 0;
      for (int y = 0; y < size_y; y++)
         for (int x = 0; x < size_x; x++)
            if (shapesize = 0, FillObject(x, y))
            {
               bitmap_count++;
               if (shapesize < MINSHAPESIZE)
                  if (verbous)
                     printf("Object at %d, %d is too small.\n", x, y);
            }
      if (verbous)
      {
         printf("Count: %d\n", bitmap_count);
         printf("---------------------\n");
      }

   } catch (png::std_error e) {
      printf("\nCannot open file: %s\n", filename);
      ::exit(1);
   }
}


// routes the signal thru all the layers - necessary for numbering the signals
void RouteSignal(int x, int y, int sig_num, int layer)
{
   stack_x[0] = x;
   stack_y[0] = y;
   stack_layer[0] = layer;
   int stack_ptr = 1;

   while (stack_ptr > 0) {

      stack_ptr--;
      x = stack_x[stack_ptr];
      y = stack_y[stack_ptr];
      layer = stack_layer[stack_ptr];

      switch (layer) {
      case METAL:
         if (!(pombuf[y * size_x + x] & METAL)) {
            continue;
         }
         break;
      case POLYSILICON:
         if (!(pombuf[y * size_x + x] & POLYSILICON)) {
            continue;
         }
         break;
      case DIFFUSION:
         if (!(pombuf[y * size_x + x] & REAL_DIFFUSION)) {
            continue;
         }
         break;
      }

      int pomsig = 0;
      switch (layer) {
      case METAL:
         pomsig = signals_metal[y * size_x + x];
         break;
      case POLYSILICON:
         pomsig = signals_poly[y * size_x + x];
         break;
      case DIFFUSION:
         pomsig = signals_diff[y * size_x + x];
         break;
      }
      if (pomsig) {
         if (pomsig != sig_num) {
            if (verbous) {
               printf("Signal mismatch %d vs %d at %d, %d\n", sig_num, pomsig, x, y);
            }
         }
         continue;
      }

      switch (layer) {
      case METAL:
         signals_metal[y * size_x + x] = sig_num;
         break;
      case POLYSILICON:
         signals_poly[y * size_x + x] = sig_num;
         break;
      case DIFFUSION:
         signals_diff[y * size_x + x] = sig_num;
         break;
      }

      for (int i = 0; i < 4; i++) {
         int nx;
         int ny;
         switch (i) {
         case 0: nx = x - 1; ny = y    ; break;
         case 1: nx = x    ; ny = y - 1; break;
         case 2: nx = x + 1; ny = y    ; break;
         case 3: nx = x    ; ny = y + 1; break;
         }
         if (nx >= 0 && nx < size_x && ny >= 0 && ny < size_y) {
            if (stack_ptr < FILL_STACK_SIZE) {
               stack_x[stack_ptr] = nx;
               stack_y[stack_ptr] = ny;
               stack_layer[stack_ptr] = layer;
               stack_ptr++;
            } else {
               printf("Fill stack overflow in RouteSignal()\n");
               ::exit(1);
            }
         }
      }

      if (stack_ptr < FILL_STACK_SIZE) {
         // These don't actually take effect until stack_ptr is incremented
         stack_x[stack_ptr] = x;
         stack_y[stack_ptr] = y;
         switch (layer) {
         case METAL:
            if ((pombuf[y * size_x + x] & VIAS_TO_POLYSILICON)) {
               stack_layer[stack_ptr++] = POLYSILICON;
            } else if ((pombuf[y * size_x + x] & VIAS_TO_DIFFUSION)) {
               stack_layer[stack_ptr++] = DIFFUSION;
            }
            break;
         case POLYSILICON:
            if ((pombuf[y * size_x + x] & VIAS_TO_POLYSILICON)) {
               stack_layer[stack_ptr++] = METAL;
            } else if ((pombuf[y * size_x + x] & BURIED_CONTACT)) {
               stack_layer[stack_ptr++] = DIFFUSION;
            }
            break;
         case DIFFUSION:
            if ((pombuf[y * size_x + x] & VIAS_TO_DIFFUSION)) {
               stack_layer[stack_ptr++] = METAL;
            } else if ((pombuf[y * size_x + x] & BURIED_CONTACT)) {
               stack_layer[stack_ptr++] = POLYSILICON;
            }
            break;
         }
      } else {
         printf("Fill stack overflow in RouteSignal()\n");
         ::exit(1);
      }
   }
}

int gate, source, drain;
int sourcelen, drainlen, otherlen;

// helping function for setting source and drain of the transistor
void SetSourceDran(int x, int y, int value)
{
   if (!source)
   {
      source = value;
      sourcelen++;
   }
   else
   {
      if (source == value)
      {
         sourcelen++;
      }
      else
      {
         if (!drain)
         {
            drain = value;
            drainlen++;
         }
         else
         {
            if (drain == value)
            {
               drainlen++;
            }
            else
            {
               if (verbous)
                  printf("Transistor has more than 3 terminals at %d %d.\n", x, y);
            }
         }
      }
   }
}

// gets the value of 8 transistors - for listing purposes
int GetRegVal(unsigned int reg[])
{
   int pomvalue = 0;
   for (int i = 7; i >= 0; i--)
      pomvalue = (pomvalue << 1) | (transistors[reg[i]].IsOn() & 1);
   return pomvalue;
}

void WriteTransCoords(int bit7, int bit6, int bit5, int bit4, int bit3, int bit2, int bit1, int bit0)
{
   printf("[%d, %d], ", transistors[bit7].x, transistors[bit7].y);
   printf("[%d, %d], ", transistors[bit6].x, transistors[bit6].y);
   printf("[%d, %d], ", transistors[bit5].x, transistors[bit5].y);
   printf("[%d, %d], ", transistors[bit4].x, transistors[bit4].y);
   printf("[%d, %d], ", transistors[bit3].x, transistors[bit3].y);
   printf("[%d, %d], ", transistors[bit2].x, transistors[bit2].y);
   printf("[%d, %d], ", transistors[bit1].x, transistors[bit1].y);
   printf("[%d, %d]", transistors[bit0].x, transistors[bit0].y);
}

// finds the transistor by coordinates - the coordinations must be upper - left corner ie the most top (first) and most left (second) corner
int FindTransistor(unsigned int x, unsigned int y)
{
   for (unsigned int i = 0; i < transistors.size(); i++)
      if (transistors[i].x == x && transistors[i].y == y)
         return i;
   printf("--- Error --- Transistor at %d, %d not found.\n", x, y);
   return -1;
}

int FindSignalX(int i)
{
   int sig = -1;
   Transistor t = transistors[i];
   return t.gate;
}

int FindSignal(int i)
{
   int sig = -1;
   Transistor t = transistors[i];
   if (t.drain == SIG_GND || t.drain == SIG_VCC) {
      sig = t.source;
      printf("t%d: using source because drain is fixed at %d\n", i, (t.drain == SIG_VCC ? 1 : 0));
   } else if (t.source == SIG_GND || t.source == SIG_VCC) {
      sig = t.drain;
      printf("t%d: using drain because source is fixed at %d\n", i, (t.source == SIG_VCC ? 1 : 0));
   }
   if (sig < 0) {
      for (int j = 0; j < t.drainconnections.size(); j++) {
         int x = t.drainconnections[j].index;
         printf("t%d.drain connects to t%d\n", i, x);
         Transistor t2 = transistors[x];
         if (t2.source == t2.gate && t2.source == t.drain && t2.drain == SIG_VCC) {
            sig = t.drain;
            printf("t%d: using drain because net connects to pullup t%d\n", i, x);
            break;
         }
      }
   }
   if (sig < 0) {
      for (int j = 0; j < t.sourceconnections.size(); j++) {
         int x = t.sourceconnections[j].index;
         printf("t%d.source connects to t%d\n", i, x);
         Transistor t2 = transistors[x];
         if (t2.source == t2.gate && t2.source == t.source && t2.drain == SIG_VCC) {
            sig = t.source;
            printf("t%d: using source because net connects to pullup t%d\n", i, x);
            break;
         }
      }
   }
   if (sig < 0 || sig == SIG_GND || sig == SIG_VCC) {
      printf("Failed to find output of transistor %d, defaulting to the drain\n", i);
      sig = t.drain;
   }
   printf("t%d: Using signal %d\n", i, sig);
   return sig;
}

void CheckTransistor(int x, int y)
{
   if ((pombuf[y * size_x + x] & (TRANSISTORS | TEMPORARY)) != TRANSISTORS)
      return;

   if (!signals_poly[y * size_x + x])
      if (verbous)
         printf("Transistor with no signal in gate at: %d %d.\n", x, y);
   if (!gate)
      gate = signals_poly[y * size_x + x];
   if (gate != signals_poly[y * size_x + x])
      if (verbous)
         printf("Ambiguous signals in poly for transistor at: %d %d.\n", x, y);

   pombuf[y * size_x + x] |= TEMPORARY;
   shapesize++;

   if (x)
   {
      if ((pombuf[y * size_x + x - 1] & TRANSISTORS))
      {
         CheckTransistor(x - 1, y);
      }
      else
      {
         if ((pombuf[y * size_x + x - 1] & DIFFUSION))
            SetSourceDran(x, y, signals_diff[y * size_x + x - 1]);
         else
            otherlen++;
      }
   }
   if (y)
   {
      if ((pombuf[(y - 1) * size_x + x] & TRANSISTORS))
      {
         CheckTransistor(x, y - 1);
      }
      else
      {
         if ((pombuf[(y - 1) * size_x + x] & DIFFUSION))
            SetSourceDran(x, y, signals_diff[(y - 1) * size_x + x]);
         else
            otherlen++;
      }
   }
   if (x < size_x - 1)
   {
      if ((pombuf[y * size_x + x + 1] & TRANSISTORS))
      {
         CheckTransistor(x + 1, y);
      }
      else
      {
         if ((pombuf[y * size_x + x + 1] & DIFFUSION))
            SetSourceDran(x, y, signals_diff[y * size_x + x + 1]);
         else
            otherlen++;
      }
   }
   if (y < size_y - 1)
   {
      if ((pombuf[(y + 1) * size_x + x] & TRANSISTORS))
      {
         CheckTransistor(x, y + 1);
      }
      else
      {
         if ((pombuf[(y + 1) * size_x + x] & DIFFUSION))
            SetSourceDran(x, y, signals_diff[(y + 1) * size_x + x]);
         else
            otherlen++;
      }
   }
}

void SetupPad(int x, int y, int signalnum, int padtype)
{
   Pad tmppad;

   tmppad.x = x;
   tmppad.y = y;
   tmppad.origsignal = signalnum;
   tmppad.type = padtype;
   pads.push_back(tmppad);
   RouteSignal(tmppad.x, tmppad.y, tmppad.origsignal, METAL);
}


void write_transdefs_file() {
      // The format here is
   //   name
   //   gate,c1,c2
   //   bb (bounding box: xmin, xmax, ymin, ymax)
   //   geometry (unused) (width1, width2, length, #segments, area)
   //   weak (boolean) (marks weak transistors, whether pullups or pass gates)
   //

   FILE* trfile = ::fopen("transdefs.js", "wb");
   ::fputs("var transdefs = [\n", trfile);
   for (unsigned int i = 0; i < transistors.size(); i++)
   {
      int weak = 0;
      // ['t1',1646,13,663,[560,721,2656,2730],[415,415,11,5,4566],false],
      Transistor t = transistors[i];
      // Omit transistors that are pullups
      if (t.source != t.gate || t.drain != SIG_VCC) {
         if (t.drain == SIG_VCC && t.gate == SIG_VCC) {
            printf("Warning: t%d is a enhancement pullup (g=%d, s=%d, d=%d) - marking as weak\n", i, t.gate, t.source, t.drain);
            weak = 1;
         } else {
            if (t.source == SIG_GND && t.gate == SIG_GND) {
               printf("Warning: t%d is a protection diode (g=%d, s=%d, d=%d) - excluding\n", i, t.gate, t.source, t.drain);
               continue;
            }
            if (t.source == 0) {
               printf("Warning: t%d has disconnected source (area = %d) - excluding\n", i, (int) t.area);
               continue;
            }
            if (t.drain == 0) {
               printf("Warning: t%d has disconnected drain (area = %d) - excluding\n", i, (int) t.area);
               continue;
            }
            if (t.gate == 0) {
               printf("Warning: t%d has disconnected gate (area = %d) - excluding\n", i, (int) t.area);
               continue;
            }
            if (t.source == t.gate || t.source == t.drain || t.gate == t.drain) {
               printf("Warning: t%d has unusual connections (g=%d, s=%d, d=%d)\n", i, t.gate, t.source, t.drain);
            }
         }
         int gate = t.gate;
         if (t.depletion) {
            printf("Warning: t%d is depletion mode (g=%d, s=%d, d=%d) but not a pullup; trap? forcing gate to VCC in netlist\n", i, t.gate, t.source, t.drain);
            gate = SIG_VCC;
         }
         ::fprintf(trfile, "['t%d',%d,%d,%d,", i, gate, t.source, t.drain);
         ::fprintf(trfile, "[%d,%d,%d,%d],", t.x, t.y, 1, 1);
         ::fprintf(trfile, "[%d,%d,%d,%d,%d],%s,],\n", 1, 1, 1, 1, (int) t.area, (weak ? "true" : "false"));
      } else {
         if (!t.depletion) {
            printf("Warning: t%d is not depletion mode and is being excluded\n", i);
         }
      }
   }
   ::fputs("]\n", trfile);
   ::fclose(trfile);
}


void write_segdefs_file_simple() {
   FILE* segfile = ::fopen("segdefs.js", "wb");
   ::fputs("var segdefs = [\n", segfile);
   for (unsigned int i = 0; i < signals.size(); i++)
   {
      // [   0,'+',1,5391,8260,5391,8216,5357,8216,5357,8260],
      int pullup = '-';
      if (i != SIG_GND || i != SIG_VCC) {
         // Work out if signal is a depletion
         for (unsigned int j = 0; j < signals[i].connections.size(); j++) {
            Transistor t = transistors[signals[i].connections[j].index];
            if (t.source == i && t.gate == i && t.drain == SIG_VCC) {
               pullup = '+';
               if (!t.depletion) {
                  printf("Warning: sig %d / transistor %d expected to be depletion\n", i, j);
               }
            }
         }
      }
      ::fprintf(segfile, "[ %d,'%c',1],\n", i, pullup);
   }
   ::fputs("]\n", segfile);
   ::fclose(segfile);
}

#define EXCLUSION 50

#define CONSTRAINT_NONE     0
#define CONSTRAINT_GND      1
#define CONSTRAINT_VCC      2
#define CONSTRAINT_SWITCHED 3

// Layers:
// 0 - Metal
// 1 - Switched Diffusion (signal != SIG_GND && signal != SIG_VCC)
// 2 - Input Diode
// 3 - Grounded Diffusion (signal == SIG_GND)
// 4 - Powered Diffusion (signal == SIG_VCC)
// 5 - Polysilicon

// [   0,'+',1,5391,8260,5391,8216,5357,8216,5357,8260],

// +1 to turn right
// -1 to turn left

#define DIR_R 0
#define DIR_D 1
#define DIR_L 2
#define DIR_U 3

void trace_boundary(FILE *segfile, int layer, uint16_t *sigs, int start_x, int start_y) {
   int sig = sigs[start_y * size_x + start_x];
   ::fprintf(segfile, "[ %d,'%c',%d", sig, (signals[sig].pullup ? '+' : '-'), layer);


   if (layer > 0) {


   // printf("tracing signal %d starting at %d, %d\n", sig, start_x, start_y);

   // Trace the boundary using the "square tracing" algorithm

   // Starting point is the top left corner, so start facing right

   int len      = 0;
   int x        = start_x;
   int y        = start_y;
   int dir      = DIR_R;
   int vertex_x = x;
   int vertex_y = y;
   int last_x   = x;
   int last_y   = y;

   // Output the start point, and mark as visited
   ::fprintf(segfile, ",%d,%d", x, y);
   sigs[y * size_x + x] |= 0x8000;

   do {
      // Step forwards
      switch (dir) {
      case DIR_U:
         y = y - 1;
         break;
      case DIR_D:
         y = y + 1;
         break;
      case DIR_L:
         x = x - 1;
         break;
      case DIR_R:
         x = x + 1;
         break;
      }

      // Test the next point, if within the image
      int point = 0;
      if (x >= EXCLUSION && x < size_x - EXCLUSION && y >= EXCLUSION && y < size_y - EXCLUSION) {
         point = sigs[y * size_x + x];
      }

      // If point is on the boundary, mark as visited
      if (point) {
         sigs[y * size_x + x] |= 0x8000;
      }

      // Turn accordingly
      if (point) {
         // Turn left
         if (dir == 0) {
            dir = 3;
         } else {
            dir = dir - 1;
         }
      } else {
         // Turn right
         if (dir == 3) {
            dir = 0;
         } else {
            dir = dir + 1;
         }
      }

      // Consider outputting the point
      if (point) {
         // TODO: deal with diagnonals
         if (x != vertex_x && y != vertex_y) {
            // Output the last point as a vertex
            ::fprintf(segfile, ",%d,%d", last_x, last_y);
            // printf("%d %d\n", last_x, last_y);
            vertex_x = last_x;
            vertex_y = last_y;
         }
         last_x = x;
         last_y = y;
      }
      len++;
   } while (len < 100000 && (x != start_x || y != start_y));

   printf("len = %d\n", len);

   }

   ::fprintf(segfile, "],\n");

   //::exit(1);
}


void write_layer_segments(FILE *segfile, int layer, uint16_t *sigs, int constraint) {
   for (int y = EXCLUSION; y < size_y - EXCLUSION; y++) {
      int last_sig = 0;
      for (int x = EXCLUSION; x < size_x - EXCLUSION; x++) {
         int sig = sigs[y * size_x + x];
         // Identify the top left corner of an object
         if (last_sig == 0 && sig > 0 && sig < 0x8000) {
            bool found;
            switch (constraint) {
            case CONSTRAINT_NONE:
               found = true;
               break;
            case CONSTRAINT_GND:
               found = sig == SIG_GND;
               break;
            case CONSTRAINT_VCC:
               found = sig == SIG_VCC;
               break;
            case CONSTRAINT_SWITCHED:
               found = sig != SIG_GND && sig != SIG_VCC;
               break;
            default:
               found = false;
            }
            if (found) {
               trace_boundary(segfile, layer, sigs, x, y);
            }
         }
         last_sig = sig;
      }
   }
   uint16_t *sp = sigs;
   for (int i = 0; i < size_x * size_y; i++) {
      *sp &= 0x7FFF;
      sp++;
   }
}


void  write_segdefs_file() {
   // First, work out whether each signal needs to be marked with a depletion pullup
   for (unsigned int i = FIRST_SIGNAL; i < signals.size(); i++) {
      // Work out if signal is a depletion
      for (unsigned int j = 0; j < signals[i].connections.size(); j++) {
         Transistor t = transistors[signals[i].connections[j].index];
         if (t.source == i && t.gate == i && t.drain == SIG_VCC) {
            signals[i].pullup = true;
            if (!t.depletion) {
               printf("Warning: sig %d / transistor %d expected to be depletion\n", i, j);
            }
         }
      }
   }

   FILE* segfile = ::fopen("segdefs.js", "wb");
   ::fputs("var segdefs = [\n", segfile);
   write_layer_segments(segfile, 0, signals_metal, CONSTRAINT_NONE);
   write_layer_segments(segfile, 1, signals_diff,  CONSTRAINT_SWITCHED);
   write_layer_segments(segfile, 3, signals_diff,  CONSTRAINT_GND);
   write_layer_segments(segfile, 4, signals_diff,  CONSTRAINT_VCC);
   write_layer_segments(segfile, 5, signals_poly,  CONSTRAINT_NONE);
   ::fputs("]\n", segfile);
   ::fclose(segfile);
}


// Everything starts here
int main(int argc, char *argv[])
{
   int64_t duration = GetTickCount();

   ZeroMemory(&memory[0], 65536 * sizeof(uint8_t));
   ZeroMemory(&ports[0], 256 * sizeof(uint8_t));

   // Simulated Z80 program

   // 0x00,                    // NOP
   // 0x3C,                    // INC A
   // 0x04,                    // INC B
   // 0x14,                    // INC D
   // 0x24,                    // INC H
   // 0xEB,                    // EXX DE,HL
   // 0x00,                    // NOP
   // 0x3C,                    // INC A
   // 0x04,                    // INC B
   // 0x14,                    // INC D
   // 0x24,                    // INC H
   // 0xD9,                    // EXX
   // 0x00,                    // NOP
   // 0x3C,                    // INC A
   // 0x04,                    // INC B
   // 0x14,                    // INC D
   // 0x24,                    // INC H
   // 0xEB,                    // EXX DE,HL
   // 0x00,                    // NOP
   // 0x3C,                    // INC A
   // 0x04,                    // INC B
   // 0x14,                    // INC D
   // 0x24,                    // INC H
   // 0x08,                    // EXX AF,AF'
   // 0x00,                    // NOP
   // 0x3C,                    // INC A
   // 0x04,                    // INC B
   // 0x14,                    // INC D
   // 0x24,                    // INC H
   // 0x00,                    // NOP
   // 0x00,                    // NOP
   // 0x00,                    // NOP
   // 0x21, 0x00, 0x01,        // LD HL,$0100
   // 0x36, 0xCC,              // LD (HL),$CC
   // 0x00,                    // NOP
   // 0x7E,                    // LD A, (HL)
   // 0x00,                    // NOP
   // 0x21, 0x34, 0x12,        // LD HL,$1234
   // 0x31, 0xfe, 0xdc,        // LD SP,0xDCFE
   // 0xe5,                    // PUSH HL
   // 0x21, 0x78, 0x56,        // LD HL,$5678
   // 0xe3,                    // EX (SP),HL
   // 0xdd, 0x21, 0xbc,0x9a,   // LD IX, 0x9ABC
   // 0xdd, 0xe3,              // EX (SP),IX
   // 0x76                     // HALT

   int mem_addr = 0;

   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x3C;
   memory[mem_addr++] = 0x04;
   memory[mem_addr++] = 0x14;
   memory[mem_addr++] = 0x24;
   memory[mem_addr++] = 0xEB;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x3C;
   memory[mem_addr++] = 0x04;
   memory[mem_addr++] = 0x14;
   memory[mem_addr++] = 0x24;
   memory[mem_addr++] = 0xD9;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x3C;
   memory[mem_addr++] = 0x04;
   memory[mem_addr++] = 0x14;
   memory[mem_addr++] = 0x24;
   memory[mem_addr++] = 0xEB;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x3C;
   memory[mem_addr++] = 0x04;
   memory[mem_addr++] = 0x14;
   memory[mem_addr++] = 0x24;
   memory[mem_addr++] = 0x08;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x3C;
   memory[mem_addr++] = 0x04;
   memory[mem_addr++] = 0x14;
   memory[mem_addr++] = 0x24;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x21;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x01;
   memory[mem_addr++] = 0x36;
   memory[mem_addr++] = 0xcc;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x7e;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x21;
   memory[mem_addr++] = 0x34;
   memory[mem_addr++] = 0x12;
   memory[mem_addr++] = 0x31;
   memory[mem_addr++] = 0xfe;
   memory[mem_addr++] = 0xdc;
   memory[mem_addr++] = 0xe5;
   memory[mem_addr++] = 0x21;
   memory[mem_addr++] = 0x78;
   memory[mem_addr++] = 0x56;
   memory[mem_addr++] = 0xe3;
   memory[mem_addr++] = 0xdd;
   memory[mem_addr++] = 0x21;
   memory[mem_addr++] = 0xbc;
   memory[mem_addr++] = 0x9a;
   memory[mem_addr++] = 0xdd;
   memory[mem_addr++] = 0xe3;
   memory[mem_addr++] = 0x76;

   if (argc < 2)
   {
      printf("Need filename as argument.\n");
      return 0;
   }

   FILE *outfile = NULL;
   //outfile = ::fopen("outfile.txt", "wb");

   for (int i = 2; i < argc; i++)
   {
      if (!::strcmp(argv[i], "-verbous"))
         verbous = true;
      else if (!::strcmp(argv[i], "-quiet"))
         verbous = false;
      else if (!::strcmp(argv[i], "-outfile"))
      {
         i++;
         if (argc == i)
         {
            printf("Filename of outfile expected.\n");
         }
         else
         {
            if (outfile)
               fclose(outfile);
            outfile = ::fopen(argv[i], "wb");
            if (!outfile)
               printf("Couldn't open %s as outfile.\n", argv[i]);
         }
      }
      else if (!::strcmp(argv[i], "-locale"))
      {
         i++;
         if (argc == i)
         {
            printf("Locale specifier expected.\n");
         }
         else
         {
            char *tmp = setlocale(LC_ALL, argv[i]);
            if (!tmp)
               printf("Couldn't set locale %s.\n", argv[i]);
         }
      }
      else if (!::strcmp(argv[i], "-divisor"))
      {
         i++;
         if (argc == i)
         {
            printf("Divisor value (100 - 10000) expected.\n");
         }
         else
         {
            int pomdivisor = atoi(argv[i]);
            if (pomdivisor < 100 || pomdivisor > 10000)
               printf("Divisor out of limit (100 - 10000): %d.\n", pomdivisor);
            else
               DIVISOR = pomdivisor;
         }
      }
      else if (!::strcmp(argv[i], "-memfile"))
      {
         i++;
         if (argc == i)
         {
            printf("Filename of memfile expected.\n");
         }
         else
         {
            i++;
            if (argc == i)
            {
               printf("Expected destination address memfile.\n");
            }
            else
            {
               int pomaddress = atoi(argv[i]);
               if (pomaddress < 0 || pomaddress > 65536)
                  printf("Destination address out of limit (0 - 65535): %d.\n", pomaddress);
               else
               {
                  FILE *memfile = ::fopen(argv[i-1], "rb");
                  if (!memfile)
                     printf("Couldn't open %s as memfile.\n", argv[i-1]);
                  ::fseek(memfile, 0, SEEK_END);
                  int filelen = ::ftell(memfile);
                  ::fseek(memfile, 0, SEEK_SET);
                  if (pomaddress + filelen > 65536)
                  {
                     printf("Memfile %s too long for specified destination address (%d). Only part will be read.\n", argv[i-1], pomaddress);
                     filelen = 65536 - pomaddress;
                  }
                  if (::fread(&memory[pomaddress], 1, filelen, memfile) <= 0) {
                     printf("Couldn't read %s as memfile.\n", argv[i-1]);
                  }
                  ::fclose(memfile);
               }
            }
         }
      }
      else
      {
         printf("Unknown switch %s.\n", argv[i]);
      }
   }

#ifdef DMB_THREAD
   threadList = new HANDLE[thread_count];
#endif

   // Loads the layers to pombuffer[]
   CheckFile(argv[1], METAL);
   CheckFile(argv[1], VIAS);
   CheckFile(argv[1], PADS);
   CheckFile(argv[1], POLYSILICON);
   CheckFile(argv[1], DIFFUSION);
   CheckFile(argv[1], BURIED);
   CheckFile(argv[1], ION_IMPLANTS);

   // basic checks of the layers
   for (int y = 0; y < size_y; y++)
      for (int x = 0; x < size_x; x++)
         if ((pombuf[y * size_x + x] & (VIAS | METAL)) == VIAS)
            if (verbous)
               printf("Via without metal at: %d %d.\n", x, y);

   for (int y = 0; y < size_y; y++)
      for (int x = 0; x < size_x; x++)
         if ((pombuf[y * size_x + x] & (VIAS | DIFFUSION | POLYSILICON)) == VIAS)
            if (verbous)
               printf("Via to nowhere at: %d %d.\n", x, y);

   for (int y = 0; y < size_y; y++)
      for (int x = 0; x < size_x; x++)
         if ((pombuf[y * size_x + x] & (VIAS | METAL | DIFFUSION | POLYSILICON)) == (VIAS | METAL | DIFFUSION | POLYSILICON))
            if (verbous)
               printf("Via both to polysilicon and diffusion at: %d %d.\n", x, y);

   for (int y = 0; y < size_y; y++)
      for (int x = 0; x < size_x; x++)
         if ((pombuf[y * size_x + x] & (VIAS | BURIED)) == (VIAS | BURIED))
            if (verbous)
               printf("Buried under via at: %d %d.\n", x, y);

   int structure_count = 0;
   type = TRANSISTORS;
   type2 = POLYSILICON | DIFFUSION | BURIED;
   type3 = POLYSILICON | DIFFUSION;
   for (int y = 0; y < size_y; y++)
      for (int x = 0; x < size_x; x++)
         if (shapesize = 0, FillStructure(x, y))
         {
            structure_count++;
            if (shapesize < 25)
               if (verbous)
                  printf("Transistor at %d, %d is too small.\n", x, y);
         }
   if (verbous)
   {
      printf("---------------------\n");
      printf("Transistor count: %d\n", structure_count);
   }

   structure_count = 0;
   type = VIAS_TO_POLYSILICON;
   type2 = VIAS | POLYSILICON | DIFFUSION;
   type3 = VIAS | POLYSILICON;
   for (int y = 0; y < size_y; y++)
      for (int x = 0; x < size_x; x++)
         if (shapesize = 0, FillStructure(x, y))
         {
            structure_count++;
            if (shapesize < 25)
               if (verbous)
                  printf("Via to poly at %d, %d is too small.\n", x, y);
         }
   if (verbous)
      printf("Vias to poly count: %d\n", structure_count);

   structure_count = 0;
   type = VIAS_TO_DIFFUSION;
   type2 = VIAS | POLYSILICON | DIFFUSION;
   type3 = VIAS | DIFFUSION;
   for (int y = 0; y < size_y; y++)
      for (int x = 0; x < size_x; x++)
         if (shapesize = 0, FillStructure(x, y))
         {
            structure_count++;
            if (shapesize < 25)
               if (verbous)
                  printf("Via to diffusion at %d, %d is too small.\n", x, y);
         }
   if (verbous)
      printf("Vias to diffusion count: %d\n", structure_count);

   structure_count = 0;
   type = BURIED_CONTACT;
   type2 = BURIED | POLYSILICON | DIFFUSION;
   type3 = BURIED | POLYSILICON | DIFFUSION;
   for (int y = 0; y < size_y; y++)
      for (int x = 0; x < size_x; x++)
         if (shapesize = 0, FillStructure(x, y))
         {
            structure_count++;
            if (shapesize < 25)
               if (verbous)
                  printf("Buried contact at %d, %d is too small.\n", x, y);
         }
   if (verbous)
      printf("Buried contacts count: %d\n", structure_count);

   structure_count = 0;
   type = REAL_DIFFUSION;
   type2 = DIFFUSION | TRANSISTORS;
   type3 = DIFFUSION;
   for (int y = 0; y < size_y; y++)
      for (int x = 0; x < size_x; x++)
         if (shapesize = 0, FillStructure(x, y))
         {
            structure_count++;
            if (shapesize < 25)
               if (verbous)
                  printf("Real diffusion at %d, %d is too small.\n", x, y);
         }
   if (verbous)
      printf("Real diffusions count: %d\n", structure_count);

   type = TEMPORARY;
   type2 = ION_IMPLANTS;
   type3 = TRANSISTORS;
   for (int y = 0; y < size_y; y++)
      for (int x = 0; x < size_x; x++)
         if (objectfound = false, FillStructureCheck(x, y))
            if (!objectfound)
               if (verbous)
                  printf("Ion implant without transistor at: %d %d.\n", x, y);
   ClearTemporary();

   signals_metal = new uint16_t[size_x * size_y];
   ZeroMemory(&signals_metal[0], size_x * size_y * sizeof(uint16_t));
   signals_poly = new uint16_t[size_x * size_y];
   ZeroMemory(&signals_poly[0], size_x * size_y * sizeof(uint16_t));
   signals_diff = new uint16_t[size_x * size_y];
   ZeroMemory(&signals_diff[0], size_x * size_y * sizeof(uint16_t));

   // Routes first few named signals
   RouteSignal(250, 2600, SIG_GND, METAL);
   RouteSignal(4450, 2500, SIG_VCC, METAL);
// RouteSignal(6621, 3217, SIG_PHI1, METAL);
// RouteSignal(6602, 3184, SIG_PHI2, METAL);
// RouteSignal(6422, 4464, SIG_RESET, METAL);

   // Finds all pads and sets its type

   Pad tmppad;

   SetupPad(4477, 4777, PAD_CLK, PAD_INPUT);
   SetupPad(200, 500, PAD__RESET, PAD_INPUT);
   SetupPad(3200, 250, PAD__HALT, PAD_OUTPUT);

   SetupPad(400, 250, PAD__WAIT, PAD_INPUT);
   SetupPad(4250, 250, PAD__INT, PAD_INPUT);
   SetupPad(4000, 250, PAD__NMI, PAD_INPUT);
   SetupPad(200, 300, PAD__BUSRQ, PAD_INPUT);

   SetupPad(300, 1700, PAD__M1, PAD_OUTPUT);
   SetupPad(300, 1900, PAD__RFSH, PAD_OUTPUT);
   SetupPad(2100, 250, PAD__RD, PAD_OUTPUT);
   SetupPad(1850, 250, PAD__WR, PAD_OUTPUT);
   SetupPad(2850, 250, PAD__MREQ, PAD_OUTPUT);
   SetupPad(2450, 250, PAD__IORQ, PAD_OUTPUT);

   SetupPad(600, 250, PAD__BUSAK, PAD_OUTPUT);

   SetupPad(3950, 4800, PAD_A15, PAD_OUTPUT);
   SetupPad(3750, 4800, PAD_A14, PAD_OUTPUT);
   SetupPad(3250, 4800, PAD_A13, PAD_OUTPUT);
   SetupPad(3000, 4800, PAD_A12, PAD_OUTPUT);

   SetupPad(2500, 4800, PAD_A11, PAD_OUTPUT);
   SetupPad(2300, 4750, PAD_A10, PAD_OUTPUT);
   SetupPad(1750, 4750, PAD_A9, PAD_OUTPUT);
   SetupPad(1500, 4750, PAD_A8, PAD_OUTPUT);

   SetupPad(1050, 4750, PAD_A7, PAD_OUTPUT);
   SetupPad(850, 4750, PAD_A6, PAD_OUTPUT);
   SetupPad(350, 4750, PAD_A5, PAD_OUTPUT);
   SetupPad(300, 4500, PAD_A4, PAD_OUTPUT);

   SetupPad(300, 4000, PAD_A3, PAD_OUTPUT);
   SetupPad(300, 3750, PAD_A2, PAD_OUTPUT);
   SetupPad(300, 3250, PAD_A1, PAD_OUTPUT);
   SetupPad(300, 2850, PAD_A0, PAD_OUTPUT);

   SetupPad(4500, 1700, PAD_D7, PAD_BIDIRECTIONAL);
   SetupPad(4500, 3200, PAD_D6, PAD_BIDIRECTIONAL);
   SetupPad(4500, 3900, PAD_D5, PAD_BIDIRECTIONAL);
   SetupPad(4500, 4450, PAD_D4, PAD_BIDIRECTIONAL);
   SetupPad(4500, 4150, PAD_D3, PAD_BIDIRECTIONAL);
   SetupPad(4500, 1900, PAD_D2, PAD_BIDIRECTIONAL);
   SetupPad(4500, 250, PAD_D1, PAD_BIDIRECTIONAL);
   SetupPad(4500, 500, PAD_D0, PAD_BIDIRECTIONAL);

   nextsignal = FIRST_SIGNAL;

   // finds the rest of pads - as all the pads are already numbered there is no pad left
   for (int y = 0; y < size_y; y++)
   {
      for (int x = 0; x < size_x; x++)
      {
         if (!signals_metal[y * size_x + x] && (pombuf[y * size_x + x] & PADS))
         {
            if (verbous)
               printf("*** Pad at x:%d y:%d signal:%d\n", x, y, nextsignal);
            RouteSignal(x, y, nextsignal, METAL);
            nextsignal++;
         }
      }
   }
   if (verbous)
   {
      printf("---------------------\n");
      printf("Pads have %d signals.\n", nextsignal - 1);
   }

   // finds the rest of signals starting from METAL then in POLYSILICON and then DIFFUSION
   for (int y = 0; y < size_y; y++)
   {
      for (int x = 0; x < size_x; x++)
      {
         if (!signals_metal[y * size_x + x] && (pombuf[y * size_x + x] & METAL))
         {
            RouteSignal(x, y, nextsignal, METAL);
            nextsignal++;
         }
      }
   }
   if (verbous)
      printf("Metal has %d signals.\n", nextsignal - 1);

   for (int y = 0; y < size_y; y++)
   {
      for (int x = 0; x < size_x; x++)
      {
         if (!signals_poly[y * size_x + x] && (pombuf[y * size_x + x] & POLYSILICON))
         {
            RouteSignal(x, y, nextsignal, POLYSILICON);
            nextsignal++;
         }
      }
   }
   if (verbous)
      printf("Metal and polysilicon have %d signals.\n", nextsignal - 1);

   for (int y = 0; y < size_y; y++)
   {
      for (int x = 0; x < size_x; x++)
      {
         if (!signals_diff[y * size_x + x] && (pombuf[y * size_x + x] & REAL_DIFFUSION))
         {
            RouteSignal(x, y, nextsignal, DIFFUSION);
            nextsignal++;
         }
      }
   }
   if (verbous)
      printf("All together we have %d signals.\n", nextsignal - 1);

   // here the transistors are tested for sanity and put into the vector i.e. list of transistors is built

   int capacitors = 0,
      resistors = 0,
      protecting = 0,
      pulldowns = 0,
      pullups_dep = 0,
      pullups_enh = 0,
      diodes = 0;

   for (int y = 0; y < size_y; y++)
   {
      for (int x = 0; x < size_x; x++)
      {
         if ((pombuf[y * size_x + x] & (TRANSISTORS | TEMPORARY)) == TRANSISTORS)
         {
            gate = source = drain = 0;
            sourcelen = drainlen = otherlen = 0;
            shapesize = 0;
            CheckTransistor(x, y);
            if (!source)
               if (verbous)
                  printf("Isolated transistor at %d, %d.\n", x, y);
            if (!drain)
            {
               if (verbous)
                  printf("Capacitor at %d, %d\n", x, y);
               capacitors++;
            }
            if ((source == SIG_VCC) && (drain == SIG_GND))
               if (verbous)
                  printf("SHORTAGE at %d, %d?\n", x, y);
            if (source == SIG_VCC)
            {
               int tmp = source;
               source = drain;
               drain = tmp;
               tmp = sourcelen;
               sourcelen = drainlen;
               drainlen = tmp;
            }
            if (drain == SIG_GND)
            {
               int tmp = source;
               source = drain;
               drain = tmp;
               tmp = sourcelen;
               sourcelen = drainlen;
               drainlen = tmp;
            }
            if (gate == SIG_GND)
            {
               if (source == SIG_GND)
               {
                  protecting++;
               }
               else
               {
                  if (verbous)
                     printf("Transistor always off at %d, %d?\n", x, y);
               }
            }
            else if (gate == SIG_VCC)
            {
               resistors++;
            }
            else
            {
               if (source == SIG_GND)
                  pulldowns++;
               if (drain == SIG_VCC)
               {
                  if (gate == source)
                     pullups_dep++;
                  else
                     pullups_enh++;
               }
            }
            if ((source != SIG_GND) && (drain != SIG_VCC) && ((gate == source) || (gate == drain)))
            {
               if (verbous)
                  printf("Diode / resistor at %d, %d.\n", x, y);
               diodes++;
            }
            Transistor pomtran;
            pomtran.x = x;
            pomtran.y = y;
            pomtran.area = (float) shapesize;
            // here the area of big transistors is somewhat scaled down to speed the simulation up little bit
         // if (pomtran.area > 25.0)
         //    pomtran.area = (log(pomtran.area / 25.0f) + 1.0f) * 25.0f;
            pomtran.gate = gate;
            pomtran.source = source;
            pomtran.drain = drain;
            pomtran.sourcelen = sourcelen;
            pomtran.drainlen = drainlen;
            pomtran.otherlen = otherlen;
            if ((drain == SIG_VCC) && (source == gate))
               pomtran.depletion = true;
            if (pombuf[y * size_x + x] & ION_IMPLANTS)
               pomtran.depletion = true;
            pomtran.resist = float(otherlen) / (float(sourcelen) + float(drainlen)) + 0.9f;
            pomtran.pomchargetogo = QUANTUM / pomtran.resist;
            if (pomtran.pomchargetogo > MAXQUANTUM)
               pomtran.pomchargetogo = MAXQUANTUM;
            transistors.push_back(pomtran);
         }
      }
   }
   ClearTemporary();

   // pull-ups are put on the first positions - well bubble sort is not quite effective...
   for (unsigned int i = 0; i < transistors.size() - 1; i++)
   {
      for (unsigned int j = i + 1; j < transistors.size(); j++)
      {
         if (transistors[i].Valuate() > transistors[j].Valuate())
            std::swap(transistors[i], transistors[j]);
      }
   }

   // finds the connections between transistors and builds the list of transistors connections
   for (unsigned int i = 0; i < transistors.size(); i++)
   {
      for (unsigned int j = 0; j < transistors.size(); j++)
      {
         Connection pomcon;
         pomcon.index = j;
         if (transistors[i].gate > SIG_VCC)
         {
            if (transistors[i].gate == transistors[j].gate)
            {
               pomcon.terminal = GATE;
               transistors[i].gateconnections.push_back(pomcon);
               transistors[i].gateneighborhood += transistors[j].area;
            }
            if (transistors[i].gate == transistors[j].source)
            {
               pomcon.terminal = SOURCE;
               transistors[i].gateconnections.push_back(pomcon);
               transistors[i].gateneighborhood += transistors[j].area;
            }
            if (transistors[i].gate == transistors[j].drain)
            {
               pomcon.terminal = DRAIN;
               transistors[i].gateconnections.push_back(pomcon);
               transistors[i].gateneighborhood += transistors[j].area;
            }
         }
         if (transistors[i].source > SIG_VCC)
         {
            if (transistors[i].source == transistors[j].gate)
            {
               pomcon.terminal = GATE;
               transistors[i].sourceconnections.push_back(pomcon);
               transistors[i].sourceneighborhood += transistors[j].area;
            }
            if (transistors[i].source == transistors[j].source)
            {
               pomcon.terminal = SOURCE;
               transistors[i].sourceconnections.push_back(pomcon);
               transistors[i].sourceneighborhood += transistors[j].area;
            }
            if (transistors[i].source == transistors[j].drain)
            {
               pomcon.terminal = DRAIN;
               transistors[i].sourceconnections.push_back(pomcon);
               transistors[i].sourceneighborhood += transistors[j].area;
            }
         }
         if (transistors[i].drain > SIG_VCC)
         {
            if (transistors[i].drain == transistors[j].gate)
            {
               pomcon.terminal = GATE;
               transistors[i].drainconnections.push_back(pomcon);
               transistors[i].drainneighborhood += transistors[j].area;
            }
            if (transistors[i].drain == transistors[j].source)
            {
               pomcon.terminal = SOURCE;
               transistors[i].drainconnections.push_back(pomcon);
               transistors[i].drainneighborhood += transistors[j].area;
            }
            if (transistors[i].drain == transistors[j].drain)
            {
               pomcon.terminal = DRAIN;
               transistors[i].drainconnections.push_back(pomcon);
               transistors[i].drainneighborhood += transistors[j].area;
            }
         }
      }
      for (unsigned int k = 0; k < transistors[i].gateconnections.size(); k++)
         transistors[i].gateconnections[k].proportion = transistors[transistors[i].gateconnections[k].index].area / transistors[i].gateneighborhood;
      for (unsigned int k = 0; k < transistors[i].drainconnections.size(); k++)
         transistors[i].drainconnections[k].proportion = transistors[transistors[i].drainconnections[k].index].area / transistors[i].drainneighborhood;
      for (unsigned int k = 0; k < transistors[i].sourceconnections.size(); k++)
         transistors[i].sourceconnections[k].proportion = transistors[transistors[i].sourceconnections[k].index].area / transistors[i].sourceneighborhood;
   }

   // builds the list of connections of pads
   for (unsigned int i = 0; i < pads.size(); i++)
   {
      for (unsigned int j = 0; j < transistors.size(); j++)
      {
         Connection pomcon;
         pomcon.index = j;
         if (pads[i].origsignal > SIG_VCC)
         {
            if (pads[i].origsignal == transistors[j].gate)
            {
               pomcon.terminal = GATE;
               pads[i].connections.push_back(pomcon);
            }
            if (pads[i].origsignal == transistors[j].source)
            {
               pomcon.terminal = SOURCE;
               pads[i].connections.push_back(pomcon);
            }
            if (pads[i].origsignal == transistors[j].drain)
            {
               pomcon.terminal = DRAIN;
               pads[i].connections.push_back(pomcon);
            }
         }
      }
   }

   // builds the list of signals
   for (int i = 0; i < nextsignal; i++)
   {
      Signal pomsignal;
      if (i == SIG_VCC || i == SIG_GND)
         pomsignal.ignore = true;
      signals.push_back(pomsignal);
      for (unsigned int j = 0; j < transistors.size(); j++)
      {
         Connection pomcon;
         pomcon.index = j;
         if (i == transistors[j].gate)
         {
            pomcon.terminal = GATE;
            signals[i].signalarea += transistors[j].area;
            signals[i].connections.push_back(pomcon);
         }
         if (i == transistors[j].source)
         {
            signals[i].signalarea += transistors[j].area;
            pomcon.terminal = SOURCE;
            signals[i].connections.push_back(pomcon);
         // if (transistors[j].depletion && transistors[j].drain == SIG_VCC)
         //    signals[i].ignore = true;
         }
         if (i == transistors[j].drain)
         {
            signals[i].signalarea += transistors[j].area;
            pomcon.terminal = DRAIN;
            signals[i].connections.push_back(pomcon);
         }
      }
      for (unsigned int j = 0; j < signals[i].connections.size(); j++)
         signals[i].connections[j].proportion = transistors[signals[i].connections[j].index].area / signals[i].signalarea;
   }

   if (verbous)
   {
      printf("---------------------\n");
      printf("Num of capacitors: %d\n", capacitors);
      printf("Num of resistors: %d\n", resistors);
      printf("Num of protecting diodes: %d\n", protecting);
      printf("Num of depletion pull ups: %d\n", pullups_dep);
      printf("Num of enhancement pull ups: %d\n", pullups_enh);
      printf("Num of pull downs: %d\n", pulldowns);
      printf("Num of diodes: %d\n", diodes);

      printf("---------------------\n");
   }

   // ================================================================
   // ================================================================
   // Here the list of transistors, pads and signals is ready and done
   // ================================================================
   // =================== now it could be exported ===================
   // ================================================================
   // ================================================================

   /*for (unsigned int i = 0; i < pads.size(); i++)
   {
      printf("*** Pad at x:%d y:%d signal:%d cons:%d type:%s\n",
         pads[i].x, pads[i].y, pads[i].origsignal, pads[i].connections.size(), (pads[i].type == PAD_INPUT ? "I" : ((pads[i].type == PAD_OUTPUT) ? "O" : "B")));
   }

   for (unsigned int i = 0; i < transistors.size(); i++)
   {
      printf("*** Transistor at x:%d y:%d area:%d gate:%d source:%d drain:%d, sourcelen:%d drainlen:%d otherlen:%d gatecon:%d sourcecon:%d draincon:%d\n",
         transistors[i].x, transistors[i].y, int(transistors[i].area), transistors[i].gate, transistors[i].source, transistors[i].drain,
         transistors[i].sourcelen, transistors[i].drainlen, transistors[i].otherlen,
         transistors[i].gateconnections.size(), transistors[i].sourceconnections.size(), transistors[i].drainconnections.size());
   }*/


   // ========================================================================================================================
   // ========================================================================================================================
   // Saves the colored metal layer and vias layer - I commented it out as it did not work properly after some changes in code
   // ========================================================================================================================
   // ========================================================================================================================

   png::image<png::rgba_pixel> myImage(size_x, size_y);
   char filename[256];

   for (int y = 0; y < size_y; y++)
   {
      for (int x = 0; x < size_x; x++)
      {
         if (signals_metal[y * size_x + x] == SIG_GND)
            ::SetPixelToBitmapData(myImage, x, y, 0x0000ff);
         else if (signals_metal[y * size_x + x] == SIG_VCC)
            ::SetPixelToBitmapData(myImage, x, y, 0xff0000);
         else if (signals_metal[y * size_x + x] == PAD_CLK)
            ::SetPixelToBitmapData(myImage, x, y, 0xffffff);
         else if (pombuf[y * size_x + x] & METAL)
            ::SetPixelToBitmapData(myImage, x, y, 0x00ff00);
         else
            ::SetPixelToBitmapData(myImage, x, y, 0x000000);
      }
   }
   sprintf(filename, "%s%s", argv[1], "_metal_VCC_GND.png");
   myImage.write(filename);

   for (int y = 0; y < size_y; y++)
   {
      for (int x = 0; x < size_x; x++)
      {
         if (pombuf[y * size_x + x] & VIAS)
         {
            if (signals_metal[y * size_x + x] == SIG_GND)
               ::SetPixelToBitmapData(myImage, x, y, 0x3f3fff);
            else if (signals_metal[y * size_x + x] == SIG_VCC)
               ::SetPixelToBitmapData(myImage, x, y, 0xff3f3f);
            else if (signals_metal[y * size_x + x] == PAD_CLK)
               ::SetPixelToBitmapData(myImage, x, y, 0xffffff);
            else
               ::SetPixelToBitmapData(myImage, x, y, 0xff7f00);
         }
         else
         {
            ::SetPixelToBitmapData(myImage, x, y, 0x000000);
         }
      }
   }
   sprintf(filename, "%s%s", argv[1], "_vias_VCC_GND.png");
   myImage.write(filename);



   // -------------------------------------------------------
   // -------------------------------------------------------
   // -------------------------------------------------------
   // ---------- Here the simulation itself starts ----------
   // -------------------------------------------------------
   // -------------------------------------------------------
   // -------------------------------------------------------

   int lastadr = 0;
   int lastdata = 0;
   int pomadr = 0;
   bool pom_wr = true;
   bool pom_rd = true;
   bool pom_rst = true;
   bool pom_mreq = true;
   bool pom_iorq = true;
   bool pom_halt = true;

   int outcounter = 0;
   bool justwasoutput = false;

   duration = GetTickCount() - duration;
   if (verbous)
   {
      printf("---------------------\n");
      printf("Duration: %" PRId64 "ms\n\n", duration);
   }
   duration = GetTickCount();

   // Simulated Z80 program

   unsigned int reg_a[8], reg_f[8], reg_b[8], reg_c[8], reg_d[8], reg_e[8], reg_d2[8], reg_e2[8], reg_h[8], reg_l[8], reg_h2[8], reg_l2[8];
   unsigned int reg_w[8], reg_z[8], reg_pch[8], reg_pcl[8], reg_sph[8], reg_spl[8];
   unsigned int reg_ixh[8], reg_ixl[8], reg_iyh[8], reg_iyl[8], reg_i[8], reg_r[8];
   unsigned int reg_a2[8], reg_f2[8], reg_b2[8], reg_c2[8];

   unsigned int sig_t1 = FindTransistor(572, 1203);
   unsigned int sig_t2 = FindTransistor(831, 1895);
   unsigned int sig_t3 = FindTransistor(901, 1242);
   unsigned int sig_t4 = FindTransistor(876, 1259);
   unsigned int sig_t5 = FindTransistor(825, 1958);
   unsigned int sig_t6 = FindTransistor(921, 1211);

   unsigned int sig_m1 = FindTransistor(1051, 1057);
   unsigned int sig_m2 = FindTransistor(1014, 1165);
   unsigned int sig_m3 = FindTransistor(1018, 1319);
   unsigned int sig_m4 = FindTransistor(1027, 1300);
   unsigned int sig_m5 = FindTransistor(1014, 1243);
   unsigned int sig_m6 = FindTransistor(3180, 3026);

   // There are multiple flip flops controlling the register set:
   //
   // AF vs AF' is at the gate of the transistor at (2306,3051). (The
   // toggle flip flip is to the right of this, but I assume any part
   // of the control line works for you.)
   //
   // The exx state (BC/DE/HL vs BC'/DE'/HL') is at the transistor
   // gate at (979,3093)
   //
   // There are two DE vs HL flip flops, one for each exx state. You
   // can get them from the transistors at (1449,2971) and
   // (1193,2981).
   //
   // The combined DE vs HL control line is at the gate of the
   // transistor at (1443,3001).
   //
   // The transistors are arbitrary ones with their gates connected to
   // the control lines, so hopefully they will work for you.

   //unsigned int sig_ex_af            = FindTransistor(2306, 3051);
   //unsigned int sig_ex_bcdehl        = FindTransistor(979, 3093);
   //unsigned int sig_ex_dehl0         = FindTransistor(1449, 2971);
   //unsigned int sig_ex_dehl1         = FindTransistor(1193, 2981);
   //unsigned int sig_ex_dehl_combined = FindTransistor(1443, 3001);

   unsigned int sig_ex_af            = FindTransistor(2455, 3030);
   unsigned int sig_ex_bcdehl        = FindTransistor( 661, 3130);
   unsigned int sig_ex_dehl0         = FindTransistor(1018, 2981);
   unsigned int sig_ex_dehl1         = FindTransistor(1193, 2981);
   unsigned int sig_ex_dehl_combined = FindTransistor(1443, 3001);

   reg_pcl[0] = FindTransistor(1345, 3231);
   reg_pcl[1] = FindTransistor(1345, 3307);
   reg_pcl[2] = FindTransistor(1345, 3375);
   reg_pcl[3] = FindTransistor(1345, 3451);

   reg_pcl[4] = FindTransistor(1345, 3519);
   reg_pcl[5] = FindTransistor(1345, 3595);
   reg_pcl[6] = FindTransistor(1345, 3663);
   reg_pcl[7] = FindTransistor(1345, 3739);

   reg_pch[0] = FindTransistor(1345, 3827);
   reg_pch[1] = FindTransistor(1345, 3903);
   reg_pch[2] = FindTransistor(1345, 3971);
   reg_pch[3] = FindTransistor(1345, 4047);

   reg_pch[4] = FindTransistor(1345, 4115);
   reg_pch[5] = FindTransistor(1345, 4191);
   reg_pch[6] = FindTransistor(1345, 4259);
   reg_pch[7] = FindTransistor(1345, 4335);

   reg_r[0] = FindTransistor(1425, 3231);
   reg_r[1] = FindTransistor(1425, 3307);
   reg_r[2] = FindTransistor(1425, 3375);
   reg_r[3] = FindTransistor(1425, 3451);

   reg_r[4] = FindTransistor(1425, 3519);
   reg_r[5] = FindTransistor(1425, 3595);
   reg_r[6] = FindTransistor(1425, 3663);
   reg_r[7] = FindTransistor(1425, 3739);

   reg_i[0] = FindTransistor(1425, 3827);
   reg_i[1] = FindTransistor(1425, 3903);
   reg_i[2] = FindTransistor(1425, 3971);
   reg_i[3] = FindTransistor(1425, 4047);

   reg_i[4] = FindTransistor(1425, 4115);
   reg_i[5] = FindTransistor(1425, 4191);
   reg_i[6] = FindTransistor(1425, 4259);
   reg_i[7] = FindTransistor(1425, 4335);

   /////////////////////////////////////////////

   reg_z[0] = FindTransistor(1590, 3231);
   reg_z[1] = FindTransistor(1590, 3308);
   reg_z[2] = FindTransistor(1590, 3375);
   reg_z[3] = FindTransistor(1590, 3452);

   reg_z[4] = FindTransistor(1590, 3519);
   reg_z[5] = FindTransistor(1590, 3596);
   reg_z[6] = FindTransistor(1590, 3663);
   reg_z[7] = FindTransistor(1590, 3740);

   reg_w[0] = FindTransistor(1590, 3827);
   reg_w[1] = FindTransistor(1590, 3904);
   reg_w[2] = FindTransistor(1590, 3971);
   reg_w[3] = FindTransistor(1590, 4048);

   reg_w[4] = FindTransistor(1590, 4115);
   reg_w[5] = FindTransistor(1590, 4192);
   reg_w[6] = FindTransistor(1590, 4259);
   reg_w[7] = FindTransistor(1590, 4336);

   reg_spl[0] = FindTransistor(1670, 3231);
   reg_spl[1] = FindTransistor(1670, 3308);
   reg_spl[2] = FindTransistor(1670, 3375);
   reg_spl[3] = FindTransistor(1670, 3452);

   reg_spl[4] = FindTransistor(1670, 3519);
   reg_spl[5] = FindTransistor(1670, 3596);
   reg_spl[6] = FindTransistor(1670, 3663);
   reg_spl[7] = FindTransistor(1670, 3740);

   reg_sph[0] = FindTransistor(1670, 3827);
   reg_sph[1] = FindTransistor(1670, 3904);
   reg_sph[2] = FindTransistor(1670, 3971);
   reg_sph[3] = FindTransistor(1670, 4048);

   reg_sph[4] = FindTransistor(1670, 4115);
   reg_sph[5] = FindTransistor(1670, 4192);
   reg_sph[6] = FindTransistor(1670, 4259);
   reg_sph[7] = FindTransistor(1670, 4336);

   ///////////////////////////////////////////

   reg_iyl[0] = FindTransistor(1722, 3231);
   reg_iyl[1] = FindTransistor(1722, 3308);
   reg_iyl[2] = FindTransistor(1722, 3375);
   reg_iyl[3] = FindTransistor(1722, 3452);

   reg_iyl[4] = FindTransistor(1722, 3519);
   reg_iyl[5] = FindTransistor(1722, 3596);
   reg_iyl[6] = FindTransistor(1722, 3663);
   reg_iyl[7] = FindTransistor(1722, 3740);

   reg_iyh[0] = FindTransistor(1722, 3827);
   reg_iyh[1] = FindTransistor(1722, 3904);
   reg_iyh[2] = FindTransistor(1722, 3971);
   reg_iyh[3] = FindTransistor(1722, 4048);

   reg_iyh[4] = FindTransistor(1722, 4115);
   reg_iyh[5] = FindTransistor(1722, 4192);
   reg_iyh[6] = FindTransistor(1722, 4259);
   reg_iyh[7] = FindTransistor(1722, 4336);

   reg_ixl[0] = FindTransistor(1802, 3231);
   reg_ixl[1] = FindTransistor(1802, 3308);
   reg_ixl[2] = FindTransistor(1802, 3375);
   reg_ixl[3] = FindTransistor(1802, 3452);

   reg_ixl[4] = FindTransistor(1802, 3519);
   reg_ixl[5] = FindTransistor(1802, 3596);
   reg_ixl[6] = FindTransistor(1802, 3663);
   reg_ixl[7] = FindTransistor(1802, 3740);

   reg_ixh[0] = FindTransistor(1802, 3827);
   reg_ixh[1] = FindTransistor(1802, 3904);
   reg_ixh[2] = FindTransistor(1802, 3971);
   reg_ixh[3] = FindTransistor(1802, 4048);

   reg_ixh[4] = FindTransistor(1802, 4115);
   reg_ixh[5] = FindTransistor(1802, 4192);
   reg_ixh[6] = FindTransistor(1802, 4259);
   reg_ixh[7] = FindTransistor(1802, 4336);

   ///////////////////////////////////////////

   reg_e[0] = FindTransistor(1854, 3231);
   reg_e[1] = FindTransistor(1854, 3308);
   reg_e[2] = FindTransistor(1854, 3375);
   reg_e[3] = FindTransistor(1854, 3452);

   reg_e[4] = FindTransistor(1854, 3519);
   reg_e[5] = FindTransistor(1854, 3596);
   reg_e[6] = FindTransistor(1854, 3663);
   reg_e[7] = FindTransistor(1854, 3740);

   reg_d[0] = FindTransistor(1854, 3827);
   reg_d[1] = FindTransistor(1854, 3904);
   reg_d[2] = FindTransistor(1854, 3971);
   reg_d[3] = FindTransistor(1854, 4048);

   reg_d[4] = FindTransistor(1854, 4115);
   reg_d[5] = FindTransistor(1854, 4192);
   reg_d[6] = FindTransistor(1854, 4259);
   reg_d[7] = FindTransistor(1854, 4336);

   reg_e2[0] = FindTransistor(1933, 3231);
   reg_e2[1] = FindTransistor(1933, 3308);
   reg_e2[2] = FindTransistor(1933, 3375);
   reg_e2[3] = FindTransistor(1933, 3452);

   reg_e2[4] = FindTransistor(1933, 3519);
   reg_e2[5] = FindTransistor(1933, 3596);
   reg_e2[6] = FindTransistor(1933, 3663);
   reg_e2[7] = FindTransistor(1933, 3740);

   reg_d2[0] = FindTransistor(1933, 3827);
   reg_d2[1] = FindTransistor(1933, 3904);
   reg_d2[2] = FindTransistor(1933, 3971);
   reg_d2[3] = FindTransistor(1933, 4048);

   reg_d2[4] = FindTransistor(1933, 4115);
   reg_d2[5] = FindTransistor(1933, 4192);
   reg_d2[6] = FindTransistor(1933, 4259);
   reg_d2[7] = FindTransistor(1933, 4336);

   ///////////////////////////////////////////

   reg_l[0] = FindTransistor(1985, 3231);
   reg_l[1] = FindTransistor(1985, 3308);
   reg_l[2] = FindTransistor(1985, 3375);
   reg_l[3] = FindTransistor(1985, 3452);

   reg_l[4] = FindTransistor(1985, 3519);
   reg_l[5] = FindTransistor(1985, 3596);
   reg_l[6] = FindTransistor(1985, 3663);
   reg_l[7] = FindTransistor(1985, 3740);

   reg_h[0] = FindTransistor(1985, 3827);
   reg_h[1] = FindTransistor(1985, 3904);
   reg_h[2] = FindTransistor(1985, 3971);
   reg_h[3] = FindTransistor(1985, 4048);

   reg_h[4] = FindTransistor(1985, 4115);
   reg_h[5] = FindTransistor(1985, 4192);
   reg_h[6] = FindTransistor(1985, 4259);
   reg_h[7] = FindTransistor(1985, 4336);

   reg_l2[0] = FindTransistor(2065, 3231);
   reg_l2[1] = FindTransistor(2065, 3308);
   reg_l2[2] = FindTransistor(2065, 3375);
   reg_l2[3] = FindTransistor(2065, 3452);

   reg_l2[4] = FindTransistor(2065, 3519);
   reg_l2[5] = FindTransistor(2065, 3596);
   reg_l2[6] = FindTransistor(2065, 3663);
   reg_l2[7] = FindTransistor(2065, 3740);

   reg_h2[0] = FindTransistor(2065, 3827);
   reg_h2[1] = FindTransistor(2065, 3904);
   reg_h2[2] = FindTransistor(2065, 3971);
   reg_h2[3] = FindTransistor(2065, 4048);

   reg_h2[4] = FindTransistor(2065, 4115);
   reg_h2[5] = FindTransistor(2065, 4192);
   reg_h2[6] = FindTransistor(2065, 4259);
   reg_h2[7] = FindTransistor(2065, 4336);


   ///////////////////////////////////////////

   reg_c[0] = FindTransistor(2117, 3231);
   reg_c[1] = FindTransistor(2117, 3308);
   reg_c[2] = FindTransistor(2117, 3375);
   reg_c[3] = FindTransistor(2117, 3452);

   reg_c[4] = FindTransistor(2117, 3519);
   reg_c[5] = FindTransistor(2117, 3596);
   reg_c[6] = FindTransistor(2117, 3663);
   reg_c[7] = FindTransistor(2117, 3740);

   reg_b[0] = FindTransistor(2117, 3827);
   reg_b[1] = FindTransistor(2117, 3904);
   reg_b[2] = FindTransistor(2117, 3971);
   reg_b[3] = FindTransistor(2117, 4048);

   reg_b[4] = FindTransistor(2117, 4115);
   reg_b[5] = FindTransistor(2117, 4192);
   reg_b[6] = FindTransistor(2117, 4259);
   reg_b[7] = FindTransistor(2117, 4336);

   reg_c2[0] = FindTransistor(2196, 3231);
   reg_c2[1] = FindTransistor(2196, 3308);
   reg_c2[2] = FindTransistor(2196, 3375);
   reg_c2[3] = FindTransistor(2196, 3452);

   reg_c2[4] = FindTransistor(2196, 3519);
   reg_c2[5] = FindTransistor(2196, 3596);
   reg_c2[6] = FindTransistor(2196, 3663);
   reg_c2[7] = FindTransistor(2196, 3740);

   reg_b2[0] = FindTransistor(2196, 3827);
   reg_b2[1] = FindTransistor(2196, 3904);
   reg_b2[2] = FindTransistor(2196, 3971);
   reg_b2[3] = FindTransistor(2196, 4048);

   reg_b2[4] = FindTransistor(2196, 4115);
   reg_b2[5] = FindTransistor(2196, 4192);
   reg_b2[6] = FindTransistor(2196, 4259);
   reg_b2[7] = FindTransistor(2196, 4336);

   ///////////////////////////////////////////

   reg_f2[0] = FindTransistor(2248, 3231);
   reg_f2[1] = FindTransistor(2248, 3308);
   reg_f2[2] = FindTransistor(2248, 3375);
   reg_f2[3] = FindTransistor(2248, 3452);

   reg_f2[4] = FindTransistor(2248, 3519);
   reg_f2[5] = FindTransistor(2248, 3596);
   reg_f2[6] = FindTransistor(2248, 3663);
   reg_f2[7] = FindTransistor(2248, 3740);

   reg_a2[0] = FindTransistor(2248, 3827);
   reg_a2[1] = FindTransistor(2248, 3904);
   reg_a2[2] = FindTransistor(2248, 3971);
   reg_a2[3] = FindTransistor(2248, 4048);

   reg_a2[4] = FindTransistor(2248, 4115);
   reg_a2[5] = FindTransistor(2248, 4192);
   reg_a2[6] = FindTransistor(2248, 4259);
   reg_a2[7] = FindTransistor(2248, 4336);

   reg_f[0] = FindTransistor(2328, 3231);
   reg_f[1] = FindTransistor(2328, 3308);
   reg_f[2] = FindTransistor(2328, 3375);
   reg_f[3] = FindTransistor(2328, 3452);

   reg_f[4] = FindTransistor(2328, 3519);
   reg_f[5] = FindTransistor(2328, 3596);
   reg_f[6] = FindTransistor(2328, 3663);
   reg_f[7] = FindTransistor(2328, 3740);

   reg_a[0] = FindTransistor(2328, 3827);
   reg_a[1] = FindTransistor(2328, 3904);
   reg_a[2] = FindTransistor(2328, 3971);
   reg_a[3] = FindTransistor(2328, 4048);

   reg_a[4] = FindTransistor(2328, 4115);
   reg_a[5] = FindTransistor(2328, 4192);
   reg_a[6] = FindTransistor(2328, 4259);
   reg_a[7] = FindTransistor(2328, 4336);

// unsigned int sig_trap2 = FindTransistor(1082, 1325);
// unsigned int sig_trap2_up = FindTransistor(1096, 1325);
// unsigned int sig_trap2_down = FindTransistor(1103, 1346);
/* unsigned int sig_x = FindTransistor(4257, 1010);
   unsigned int sig_l1 = FindTransistor(4156, 918);
   unsigned int sig_l2 = FindTransistor(4167, 918);
   unsigned int sig_l3 = FindTransistor(4177, 918);
   unsigned int sig_r1 = FindTransistor(4191, 914);
   unsigned int sig_r2 = FindTransistor(4203, 914);
   unsigned int sig_r3 = FindTransistor(4216, 914);*/

// transistors[FindTransistor(2088, 2241)].depletion = true;

   printf("-------------------------------------------------------\n");
   printf("----------------- Writing netlist files----------------\n");
   printf("-------------------------------------------------------\n");

   write_transdefs_file();

   write_segdefs_file();

   FILE* nodefile = ::fopen("nodenames.js", "wb");
   ::fputs("var nodenames ={\n", nodefile);
   ::fprintf(nodefile, "vss: %d,\n",SIG_GND);
   ::fprintf(nodefile, "vcc: %d,\n",SIG_VCC);
   ::fprintf(nodefile, "clk: %d,\n",PAD_CLK);
   ::fprintf(nodefile, "ab0: %d,\n",PAD_A0);
   ::fprintf(nodefile, "ab1: %d,\n",PAD_A1);
   ::fprintf(nodefile, "ab2: %d,\n",PAD_A2);
   ::fprintf(nodefile, "ab3: %d,\n",PAD_A3);
   ::fprintf(nodefile, "ab4: %d,\n",PAD_A4);
   ::fprintf(nodefile, "ab5: %d,\n",PAD_A5);
   ::fprintf(nodefile, "ab6: %d,\n",PAD_A6);
   ::fprintf(nodefile, "ab7: %d,\n",PAD_A7);
   ::fprintf(nodefile, "ab8: %d,\n",PAD_A8);
   ::fprintf(nodefile, "ab9: %d,\n",PAD_A9);
   ::fprintf(nodefile, "ab10: %d,\n",PAD_A10);
   ::fprintf(nodefile, "ab11: %d,\n",PAD_A11);
   ::fprintf(nodefile, "ab12: %d,\n",PAD_A12);
   ::fprintf(nodefile, "ab13: %d,\n",PAD_A13);
   ::fprintf(nodefile, "ab14: %d,\n",PAD_A14);
   ::fprintf(nodefile, "ab15: %d,\n",PAD_A15);
   ::fprintf(nodefile, "_reset: %d,\n",PAD__RESET);
   ::fprintf(nodefile, "_wait: %d,\n",PAD__WAIT);
   ::fprintf(nodefile, "_int: %d,\n",PAD__INT);
   ::fprintf(nodefile, "_nmi: %d,\n",PAD__NMI);
   ::fprintf(nodefile, "_busrq: %d,\n",PAD__BUSRQ);
   ::fprintf(nodefile, "_m1: %d,\n",PAD__M1);
   ::fprintf(nodefile, "_rd: %d,\n",PAD__RD);
   ::fprintf(nodefile, "_wr: %d,\n",PAD__WR);
   ::fprintf(nodefile, "_mreq: %d,\n",PAD__MREQ);
   ::fprintf(nodefile, "_iorq: %d,\n",PAD__IORQ);
   ::fprintf(nodefile, "_rfsh: %d,\n",PAD__RFSH);
   ::fprintf(nodefile, "db0: %d,\n",PAD_D0);
   ::fprintf(nodefile, "db1: %d,\n",PAD_D1);
   ::fprintf(nodefile, "db2: %d,\n",PAD_D2);
   ::fprintf(nodefile, "db3: %d,\n",PAD_D3);
   ::fprintf(nodefile, "db4: %d,\n",PAD_D4);
   ::fprintf(nodefile, "db5: %d,\n",PAD_D5);
   ::fprintf(nodefile, "db6: %d,\n",PAD_D6);
   ::fprintf(nodefile, "db7: %d,\n",PAD_D7);
   ::fprintf(nodefile, "_halt: %d,\n",PAD__HALT);
   ::fprintf(nodefile, "_busak: %d,\n",PAD__BUSAK);

   ::fprintf(nodefile, "t1: %d,\n", FindSignalX(sig_t1));
   ::fprintf(nodefile, "t2: %d,\n", FindSignalX(sig_t2));
   ::fprintf(nodefile, "t3: %d,\n", FindSignalX(sig_t3));
   ::fprintf(nodefile, "t4: %d,\n", FindSignalX(sig_t4));
   ::fprintf(nodefile, "t5: %d,\n", FindSignalX(sig_t5));
   ::fprintf(nodefile, "t6: %d,\n", FindSignalX(sig_t6));
   ::fprintf(nodefile, "m1: %d,\n", FindSignalX(sig_m1));
   ::fprintf(nodefile, "m2: %d,\n", FindSignalX(sig_m2));
   ::fprintf(nodefile, "m3: %d,\n", FindSignalX(sig_m3));
   ::fprintf(nodefile, "m4: %d,\n", FindSignalX(sig_m4));
   ::fprintf(nodefile, "m5: %d,\n", FindSignalX(sig_m5));
   ::fprintf(nodefile, "m6: %d,\n", FindSignalX(sig_m6));
   ::fprintf(nodefile, "m6: %d,\n", FindSignalX(sig_m6));

   ::fprintf(nodefile, "ex_af: %d,\n", FindSignalX(sig_ex_af));
   ::fprintf(nodefile, "ex_bcdehl: %d,\n", FindSignalX(sig_ex_bcdehl));
   ::fprintf(nodefile, "ex_dehl0: %d,\n", FindSignalX(sig_ex_dehl0));
   ::fprintf(nodefile, "ex_dehl1: %d,\n", FindSignalX(sig_ex_dehl1));
   ::fprintf(nodefile, "ex_dehl_combined: %d,\n", FindSignalX(sig_ex_dehl_combined));

   for (int i = 0; i < 8; i++) {
      ::fprintf(nodefile, "reg_a%d: %d,\n",   i, FindSignal(reg_a[i]));
      ::fprintf(nodefile, "reg_f%d: %d,\n",   i, FindSignal(reg_f[i]));
      ::fprintf(nodefile, "reg_b%d: %d,\n",   i, FindSignal(reg_b[i]));
      ::fprintf(nodefile, "reg_c%d: %d,\n",   i, FindSignal(reg_c[i]));
      ::fprintf(nodefile, "reg_d%d: %d,\n",   i, FindSignal(reg_d[i]));
      ::fprintf(nodefile, "reg_e%d: %d,\n",   i, FindSignal(reg_e[i]));
      ::fprintf(nodefile, "reg_h%d: %d,\n",   i, FindSignal(reg_h[i]));
      ::fprintf(nodefile, "reg_l%d: %d,\n",   i, FindSignal(reg_l[i]));
      ::fprintf(nodefile, "reg_w%d: %d,\n",   i, FindSignal(reg_w[i]));
      ::fprintf(nodefile, "reg_z%d: %d,\n",   i, FindSignal(reg_z[i]));
      ::fprintf(nodefile, "reg_pch%d: %d,\n", i, FindSignal(reg_pch[i]));
      ::fprintf(nodefile, "reg_pcl%d: %d,\n", i, FindSignal(reg_pcl[i]));
      ::fprintf(nodefile, "reg_sph%d: %d,\n", i, FindSignal(reg_sph[i]));
      ::fprintf(nodefile, "reg_spl%d: %d,\n", i, FindSignal(reg_spl[i]));
      ::fprintf(nodefile, "reg_ixh%d: %d,\n", i, FindSignal(reg_ixh[i]));
      ::fprintf(nodefile, "reg_ixl%d: %d,\n", i, FindSignal(reg_ixl[i]));
      ::fprintf(nodefile, "reg_iyh%d: %d,\n", i, FindSignal(reg_iyh[i]));
      ::fprintf(nodefile, "reg_iyl%d: %d,\n", i, FindSignal(reg_iyl[i]));
      ::fprintf(nodefile, "reg_i%d: %d,\n",   i, FindSignal(reg_i[i]));
      ::fprintf(nodefile, "reg_r%d: %d,\n",   i, FindSignal(reg_r[i]));
      ::fprintf(nodefile, "reg_aa%d: %d,\n",  i, FindSignal(reg_a2[i]));
      ::fprintf(nodefile, "reg_ff%d: %d,\n",  i, FindSignal(reg_f2[i]));
      ::fprintf(nodefile, "reg_bb%d: %d,\n",  i, FindSignal(reg_b2[i]));
      ::fprintf(nodefile, "reg_cc%d: %d,\n",  i, FindSignal(reg_c2[i]));
      ::fprintf(nodefile, "reg_dd%d: %d,\n",  i, FindSignal(reg_d2[i]));
      ::fprintf(nodefile, "reg_ee%d: %d,\n",  i, FindSignal(reg_e2[i]));
      ::fprintf(nodefile, "reg_hh%d: %d,\n",  i, FindSignal(reg_h2[i]));
      ::fprintf(nodefile, "reg_ll%d: %d,\n",  i, FindSignal(reg_l2[i]));
   }
   ::fputs("}\n", nodefile);
   ::fclose(nodefile);


   // =============================
   // End of saving colored bitmaps
   // =============================

   delete pombuf;
   delete signals_diff;
   delete signals_poly;
   delete signals_metal;


   ::exit(1);

   // ======================================================================
   // ============================= Simulation =============================
   // ======================================================================

   printf("-------------------------------------------------------\n");
   printf("----------------- Starting simulation -----------------\n");
   printf("-------------------------------------------------------\n");

   int totcycles = 0;

   // maximally 2000000 iterations
   for (unsigned int i = 0; i < 1000000000; i++)
   {
      // Setting input pads
      // I commented out several tests like test of READY, SID and HOLD pads
      for (unsigned int j = 0; j < pads.size(); j++)
      {
         if (pads[j].type == PAD_INPUT)
         {
            if (pads[j].origsignal == PAD__RESET)
            {
               if (i < DIVISOR * 100)
               {
                  pads[j].SetInputSignal(SIG_GND);
                  pom_rst = true;
               }
               else
               {
                  pads[j].SetInputSignal(SIG_VCC);
                  pom_rst = false;
               }
            }
            else if (pads[j].origsignal == PAD_CLK)
            {
               int pom = i / DIVISOR;
               if (pom & 1)
                  pads[j].SetInputSignal(SIG_VCC);
               else
                  pads[j].SetInputSignal(SIG_GND);
            }
            else if (pads[j].origsignal == PAD__WAIT)
            {
                  pads[j].SetInputSignal(SIG_VCC);
            }
            else if (pads[j].origsignal == PAD__INT)
            {
                  pads[j].SetInputSignal(SIG_VCC);
            }
            else if (pads[j].origsignal == PAD__NMI)
            {
                  pads[j].SetInputSignal(SIG_VCC);
            }
            else if (pads[j].origsignal == PAD__BUSRQ)
            {
                  pads[j].SetInputSignal(SIG_VCC);
            }
         }
         else if (pads[j].type == PAD_BIDIRECTIONAL) // we have to pull data bus up or down when memory, I/O or interrupt instruction is read
         {
            {
               if (pom_rd) // nothing is read
               {
                  pads[j].SetInputSignal(SIG_FLOATING);
               }
               else
               {
                  if (!pom_mreq) // memory is read
                  {
                     if (pads[j].origsignal == PAD_D7)
                     {
                        if (memory[lastadr] & 0x80)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                     else if (pads[j].origsignal == PAD_D6)
                     {
                        if (memory[lastadr] & 0x40)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                     else if (pads[j].origsignal == PAD_D5)
                     {
                        if (memory[lastadr] & 0x20)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                     else if (pads[j].origsignal == PAD_D4)
                     {
                        if (memory[lastadr] & 0x10)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                     else if (pads[j].origsignal == PAD_D3)
                     {
                        if (memory[lastadr] & 0x08)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                     else if (pads[j].origsignal == PAD_D2)
                     {
                        if (memory[lastadr] & 0x04)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                     else if (pads[j].origsignal == PAD_D1)
                     {
                        if (memory[lastadr] & 0x02)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                     else if (pads[j].origsignal == PAD_D0)
                     {
                        if (memory[lastadr] & 0x01)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                  }
                  else if (!pom_iorq) // I/O is read
                  {
                     if (pads[j].origsignal == PAD_D7)
                     {
                        if (ports[lastadr & 0xff] & 0x80)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                     else if (pads[j].origsignal == PAD_D6)
                     {
                        if (ports[lastadr & 0xff] & 0x40)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                     else if (pads[j].origsignal == PAD_D5)
                     {
                        if (ports[lastadr & 0xff] & 0x20)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                     else if (pads[j].origsignal == PAD_D4)
                     {
                        if (ports[lastadr & 0xff] & 0x10)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                     else if (pads[j].origsignal == PAD_D3)
                     {
                        if (ports[lastadr & 0xff] & 0x08)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                     else if (pads[j].origsignal == PAD_D2)
                     {
                        if (ports[lastadr & 0xff] & 0x04)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                     else if (pads[j].origsignal == PAD_D1)
                     {
                        if (ports[lastadr & 0xff] & 0x02)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                     else if (pads[j].origsignal == PAD_D0)
                     {
                        if (ports[lastadr & 0xff] & 0x01)
                           pads[j].SetInputSignal(SIG_VCC);
                        else
                           pads[j].SetInputSignal(SIG_GND);
                     }
                  }
               }
            }
         }
         else
         {
         // some other code here ...
         }
      }
      // End of Setting input pads

      // Simulation itself
      for (unsigned int j = 0; j < transistors.size(); j++)
      {
         transistors[j].Simulate();
      }

/*    DWORD threadID;
      printf("--- Starting threads @%d\n", GetTickCount());
      for (unsigned int t = 0; t < thread_count; t++)
         threadList[t] = CreateThread(NULL, 0, ThreadSimulateTransistors, (LPVOID) t, NULL, &threadID);
      WaitForMultipleObjects(thread_count, threadList, TRUE, INFINITE);
      for (unsigned int t = 0; t < thread_count; t++)
         CloseHandle(threadList[t]);
   // WaitForSingleObject(threadList[0], INFINITE);
   // WaitForSingleObject(threadList[1], INFINITE);
      printf("--- Threads stopped @%d\n", GetTickCount());*/

      for (unsigned int j = 0; j < signals.size(); j++)
         if (!signals[j].ignore)
            signals[j].Homogenize();
      for (unsigned int j = 0; j < transistors.size(); j++)
         transistors[j].Normalize();
      // End of Simulation itself

      // Reading output pads
      if (!(i % (DIVISOR * 5)))
{
         printf("       : C// // // // AAAA AA                      \n");
         printf("       : LRH MR RW MI 1111 11AA AAAA AAAA DDDD DDDD\n");
         printf("       : KSL 1F DR QQ 5432 1098 7654 3210 7654 3210\n");
      }

      if (!(i % (DIVISOR / 5))) // writes out every 100s cycle (for output to be not too verbous)
      {
         printf("%07d: ", i);
         for (unsigned int j = 0; j < pads.size(); j++)
         {
            int pom;
            if (pads[j].type != PAD_INPUT)
            {
               if ((pads[j].type == PAD_BIDIRECTIONAL) && (pads[j].ReadInputStatus() != SIG_FLOATING))
                  pom = pads[j].ReadInputStatus();
               else
                  pom = pads[j].ReadOutputStatus();
            }
            else
            {
               pom = pads[j].ReadInputStatus();
            }
            char pom2 = pom + '0';
            if (pom == 3)
               pom2 = '.';
            else
               pom2--;
            if (pads[j].origsignal == PAD_CLK)
               printf("%c", pom2);
            if (pads[j].origsignal == PAD__RESET)
               printf("%c", pom2);
            if (pads[j].origsignal == PAD__HALT)
            {
               pom_halt = (pom2 == '1');
               printf("%c ", pom2);
            }
            if (pads[j].origsignal == PAD__M1)
               printf("%c", pom2);
            if (pads[j].origsignal == PAD__RFSH)
               printf("%c ", pom2);
            if (pads[j].origsignal == PAD__RD)
            {
               pom_rd = (pom2 == '1');
               printf("%c", pom2);
            }
            if (pads[j].origsignal == PAD__WR)
            {
               pom_wr = (pom2 == '1');
               printf("%c ", pom2);
            }
            if (pads[j].origsignal == PAD__MREQ)
            {
               pom_mreq = (pom2 == '1');
               printf("%c", pom2);
            }
            if (pads[j].origsignal == PAD__IORQ)
            {
               pom_iorq = (pom2 == '1');
               printf("%c ", pom2);
            }
            if (pads[j].origsignal == PAD_A15)
            {
               printf("%c", pom2);
               pomadr &= ~0x8000;
               pomadr |= (pom2 == '1') ? 0x8000 : 0;
            }
            if (pads[j].origsignal == PAD_A14)
            {
               printf("%c", pom2);
               pomadr &= ~0x4000;
               pomadr |= (pom2 == '1') ? 0x4000 : 0;
            }
            if (pads[j].origsignal == PAD_A13)
            {
               printf("%c", pom2);
               pomadr &= ~0x2000;
               pomadr |= (pom2 == '1') ? 0x2000 : 0;
            }
            if (pads[j].origsignal == PAD_A12)
            {
               printf("%c ", pom2);
               pomadr &= ~0x1000;
               pomadr |= (pom2 == '1') ? 0x1000 : 0;
            }
            if (pads[j].origsignal == PAD_A11)
            {
               printf("%c", pom2);
               pomadr &= ~0x0800;
               pomadr |= (pom2 == '1') ? 0x0800 : 0;
            }
            if (pads[j].origsignal == PAD_A10)
            {
               printf("%c", pom2);
               pomadr &= ~0x0400;
               pomadr |= (pom2 == '1') ? 0x0400 : 0;
            }
            if (pads[j].origsignal == PAD_A9)
            {
               printf("%c", pom2);
               pomadr &= ~0x0200;
               pomadr |= (pom2 == '1') ? 0x0200 : 0;
            }
            if (pads[j].origsignal == PAD_A8)
            {
               printf("%c ", pom2);
               pomadr &= ~0x0100;
               pomadr |= (pom2 == '1') ? 0x0100 : 0;
            }
            if (pads[j].origsignal == PAD_A7)
            {
               printf("%c", pom2);
               pomadr &= ~0x0080;
               pomadr |= (pom2 == '1') ? 0x0080 : 0;
            }
            if (pads[j].origsignal == PAD_A6)
            {
               printf("%c", pom2);
               pomadr &= ~0x0040;
               pomadr |= (pom2 == '1') ? 0x0040 : 0;
            }
            if (pads[j].origsignal == PAD_A5)
            {
               printf("%c", pom2);
               pomadr &= ~0x0020;
               pomadr |= (pom2 == '1') ? 0x0020 : 0;
            }
            if (pads[j].origsignal == PAD_A4)
            {
               printf("%c ", pom2);
               pomadr &= ~0x0010;
               pomadr |= (pom2 == '1') ? 0x0010 : 0;
            }
            if (pads[j].origsignal == PAD_A3)
            {
               printf("%c", pom2);
               pomadr &= ~0x0008;
               pomadr |= (pom2 == '1') ? 0x0008 : 0;

            }
            if (pads[j].origsignal == PAD_A2)
            {
               printf("%c", pom2);
               pomadr &= ~0x0004;
               pomadr |= (pom2 == '1') ? 0x0004 : 0;
            }
            if (pads[j].origsignal == PAD_A1)
            {
               printf("%c", pom2);
               pomadr &= ~0x0002;
               pomadr |= (pom2 == '1') ? 0x0002 : 0;
            }
            if (pads[j].origsignal == PAD_A0)
            {
               printf("%c ", pom2);
               pomadr &= ~0x0001;
               pomadr |= (pom2 == '1') ? 0x0001 : 0;
            }
            if (pads[j].origsignal == PAD_D7)
            {
               printf("%c", pom2);
               lastdata &= ~0x80;
               lastdata |= (pom2 == '1') ? 0x80 : 0;
            }
            if (pads[j].origsignal == PAD_D6)
            {
               printf("%c", pom2);
               lastdata &= ~0x40;
               lastdata |= (pom2 == '1') ? 0x40 : 0;
            }
            if (pads[j].origsignal == PAD_D5)
            {
               printf("%c", pom2);
               lastdata &= ~0x20;
               lastdata |= (pom2 == '1') ? 0x20 : 0;
            }
            if (pads[j].origsignal == PAD_D4)
            {
               printf("%c ", pom2);
               lastdata &= ~0x10;
               lastdata |= (pom2 == '1') ? 0x10 : 0;
            }
            if (pads[j].origsignal == PAD_D3)
            {
               printf("%c", pom2);
               lastdata &= ~0x08;
               lastdata |= (pom2 == '1') ? 0x08 : 0;
            }
            if (pads[j].origsignal == PAD_D2)
            {
               printf("%c", pom2);
               lastdata &= ~0x04;
               lastdata |= (pom2 == '1') ? 0x04 : 0;
            }
            if (pads[j].origsignal == PAD_D1)
            {
               printf("%c", pom2);
               lastdata &= ~0x02;
               lastdata |= (pom2 == '1') ? 0x02 : 0;
            }
            if (pads[j].origsignal == PAD_D0)
            {
               printf("%c", pom2);
               lastdata &= ~0x01;
               lastdata |= (pom2 == '1') ? 0x01 : 0;
            }
         // End of if (pads[j].type == PAD_OUTPUT)
         }

         printf(" PC:%04x", GetRegVal(reg_pch) << 8 | GetRegVal(reg_pcl));
         printf(" IR:%04x", GetRegVal(reg_i) << 8 | GetRegVal(reg_r));
         printf(" SP:%04x", GetRegVal(reg_sph) << 8 | GetRegVal(reg_spl));
         printf(" WZ:%04x", GetRegVal(reg_w) << 8 | GetRegVal(reg_z));
         printf(" IX:%04x", GetRegVal(reg_ixh) << 8 | GetRegVal(reg_ixl));
         printf(" IY:%04x", GetRegVal(reg_iyh) << 8 | GetRegVal(reg_iyl));
         printf(" HL:%04x", GetRegVal(reg_h) << 8 | GetRegVal(reg_l));
         printf(" HL':%04x", GetRegVal(reg_h2) << 8 | GetRegVal(reg_l2));
         printf(" DE:%04x", GetRegVal(reg_d) << 8 | GetRegVal(reg_e));
         printf(" DE':%04x", GetRegVal(reg_d2) << 8 | GetRegVal(reg_e2));
         printf(" BC:%04x", GetRegVal(reg_b) << 8 | GetRegVal(reg_c));
         printf(" BC':%04x", GetRegVal(reg_b2) << 8 | GetRegVal(reg_c2));
         printf(" A:%02x", GetRegVal(reg_a));
         printf(" A':%02x", GetRegVal(reg_a2));
         printf(" F:%c", (GetRegVal(reg_f) & 0x80) ? L'S' : L'.');
         printf("%c", (GetRegVal(reg_f) & 0x40) ? L'Z' : L'.');
         printf("%c", (GetRegVal(reg_f) & 0x20) ? L'5' : L'.');
         printf("%c", (GetRegVal(reg_f) & 0x10) ? L'H' : L'.');
         printf("%c", (GetRegVal(reg_f) & 0x08) ? L'3' : L'.');
         printf("%c", (GetRegVal(reg_f) & 0x04) ? L'V' : L'.');
         printf("%c", (GetRegVal(reg_f) & 0x02) ? L'N' : L'.');
         printf("%c", (GetRegVal(reg_f) & 0x01) ? L'C' : L'.');
         printf(" F':%c", (GetRegVal(reg_f2) & 0x80) ? L'S' : L'.');
         printf("%c", (GetRegVal(reg_f2) & 0x40) ? L'Z' : L'.');
         printf("%c", (GetRegVal(reg_f2) & 0x20) ? L'5' : L'.');
         printf("%c", (GetRegVal(reg_f2) & 0x10) ? L'H' : L'.');
         printf("%c", (GetRegVal(reg_f2) & 0x08) ? L'3' : L'.');
         printf("%c", (GetRegVal(reg_f2) & 0x04) ? L'V' : L'.');
         printf("%c", (GetRegVal(reg_f2) & 0x02) ? L'N' : L'.');
         printf("%c", (GetRegVal(reg_f2) & 0x01) ? L'C' : L'.');

         printf(" T:%c", (transistors[sig_t1].IsOn()) ? '1' : '.');
         printf("%c", (transistors[sig_t2].IsOn()) ? '2' : '.');
         printf("%c", (transistors[sig_t3].IsOn()) ? '3' : '.');
         printf("%c", (transistors[sig_t4].IsOn()) ? '4' : '.');
         printf("%c", (transistors[sig_t5].IsOn()) ? '5' : '.');
         printf("%c", (transistors[sig_t6].IsOn()) ? '6' : '.');

         printf(" M:%c", (transistors[sig_m1].IsOn()) ? '1' : '.');
         printf("%c", (transistors[sig_m2].IsOn()) ? '2' : '.');
         printf("%c", (transistors[sig_m3].IsOn()) ? '3' : '.');
         printf("%c", (transistors[sig_m4].IsOn()) ? '4' : '.');
         printf("%c", (transistors[sig_m5].IsOn()) ? '5' : '.');

         printf(" E:%c",         (transistors[sig_ex_af].IsOn()) ? '1' : '0');
         printf("%c",        (transistors[sig_ex_bcdehl].IsOn()) ? '1' : '0');
         printf("%c",         (transistors[sig_ex_dehl0].IsOn()) ? '1' : '0');
         printf("%c",         (transistors[sig_ex_dehl1].IsOn()) ? '1' : '0');
         printf("%c", (transistors[sig_ex_dehl_combined].IsOn()) ? '1' : '0');

      // printf(" T2:%c", (transistors[sig_trap2].IsOn()) ? 'X' : '.');
      // printf(" U:%c", (transistors[sig_trap2_up].IsOn()) ? 'X' : '.');
      // printf(" D:%c", (transistors[sig_trap2_down].IsOn()) ? 'X' : '.');

/*       printf(" X:%c", (transistors[sig_x].IsOn()) ? '#' : '.');
         printf(" L:%c", (transistors[sig_l1].IsOn()) ? '#' : '.');
         printf("%c", (transistors[sig_l2].IsOn()) ? '#' : '.');
         printf("%c", (transistors[sig_l3].IsOn()) ? '#' : '.');
         printf(" R:%c", (transistors[sig_r1].IsOn()) ? '#' : '.');
         printf("%c", (transistors[sig_r2].IsOn()) ? '#' : '.');
         printf("%c", (transistors[sig_r3].IsOn()) ? '#' : '.');*/
      // printf(" R1>>% 6.1f|% 6.1f|% 6.1f", transistors[sig_r1].gatecharge, transistors[sig_r1].draincharge, transistors[sig_r1].sourcecharge);
      // printf(" R2>>% 6.1f|% 6.1f|% 6.1f", transistors[sig_r2].gatecharge, transistors[sig_r2].draincharge, transistors[sig_r2].sourcecharge);
      // printf(" R3>>% 6.1f|% 6.1f|% 6.1f", transistors[sig_r3].gatecharge, transistors[sig_r3].draincharge, transistors[sig_r3].sourcecharge);
      // printf(" Rx>>% 5.2f|% 5.2f|% 5.2f", transistors[sig_r3].resist, transistors[sig_r3].resist, transistors[sig_r3].resist);

         if (!pom_rd && !pom_mreq && transistors[sig_m1].IsOn())
            printf(" ***** OPCODE FETCH: %04x[%02x]", pomadr, memory[pomadr]);

         if (!pom_mreq || !pom_iorq)
         {
            lastadr = pomadr; // gets the valid address
            if (!pom_rst)
            {
               if (!pom_wr)
               {
                  if (!pom_mreq)
                  {
                     memory[lastadr] = lastdata;
                     printf(" MEMORY WRITE: %04x[%02x]", lastadr, lastdata);
                  }
                  if (!pom_iorq)
                  {
                     ports[lastadr & 0xff] = lastdata;
                     printf(" I/O WRITE: %04x[%02x]", lastadr, lastdata);
                     if (!(lastadr & 0xff))
                        justwasoutput = true;
                  }
               }
               else
               {
                  if (justwasoutput)
                  {
                     justwasoutput = false;
                     if (outfile)
                        fputc(ports[0], outfile);
                  }
               }
               if (!pom_rd)
               {
                  if (!pom_mreq && !transistors[sig_m1].IsOn())
                  {
                     printf(" MEMORY READ: %04x[%02x]", lastadr, memory[lastadr]);
                  }
                  if (!pom_iorq)
                  {
                     printf(" I/O READ: %04x[%02x]", lastadr, memory[lastadr]);
                  }
               }
            }
         }

         printf("\n");

         if (!pom_halt && !pom_rst)
            outcounter++;
         else
            outcounter = 0;
         if (outcounter >= 150)
            break;

      }
      totcycles = i;
   }

   duration = GetTickCount() - duration;
   printf("---------------------\n");
   printf("Duration: %" PRId64 "ms\n", duration);
   printf("Speed of simulation: %.2fHz\n", (double(totcycles) / 2.0) / double(duration) * 1000.0 / double(DIVISOR));

   if (outfile)
      ::fclose(outfile);
#ifdef DMB_THREAD
   delete threadList;
#endif

   return 0;
}
