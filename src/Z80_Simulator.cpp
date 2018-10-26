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

unsigned int DIVISOR = 1000; // the lower the faster is clock, 1000 is lowest value I achieved
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
int x1;
int y1;
int x2;
int y2;

#define GATE 1
#define DRAIN 2
#define SOURCE 3

uint8_t memory[65536];
uint8_t ports[256];


// Globals for the names signals
unsigned int reg_a[8], reg_f[8], reg_b[8], reg_c[8], reg_d[8], reg_e[8], reg_d2[8], reg_e2[8], reg_h[8], reg_l[8], reg_h2[8], reg_l2[8];
unsigned int reg_w[8], reg_z[8], reg_pch[8], reg_pcl[8], reg_sph[8], reg_spl[8];
unsigned int reg_ixh[8], reg_ixl[8], reg_iyh[8], reg_iyl[8], reg_i[8], reg_r[8];
unsigned int reg_a2[8], reg_f2[8], reg_b2[8], reg_c2[8];

unsigned int sig_t1;
unsigned int sig_t2;
unsigned int sig_t3;
unsigned int sig_t4;
unsigned int sig_t5;
unsigned int sig_t6;

unsigned int sig_m1;
unsigned int sig_m2;
unsigned int sig_m3;
unsigned int sig_m4;
unsigned int sig_m5;
unsigned int sig_m6;

unsigned int sig_iff1;
unsigned int sig_iff2;

unsigned int sig_ex_af;
unsigned int sig_ex_bcdehl;
unsigned int sig_ex_dehl0;
unsigned int sig_ex_dehl1;
unsigned int sig_ex_dehl_combined;

unsigned int sig_alua[8];
unsigned int sig_alubus[8];
unsigned int sig_alub[8];
unsigned int sig_alulat[4];
unsigned int sig_aluout[4];
unsigned int sig_dlatch[8];
unsigned int sig_dl_dp;
unsigned int sig_dl_d;
unsigned int sig_dp_dl;
unsigned int sig_d_dl;
unsigned int sig_d_u;
unsigned int sig_instr[8];
unsigned int sig_load_ir;
unsigned int sig_pcbit[16];
unsigned int sig_pla[99];
unsigned int sig_rh_wr;
unsigned int sig_rl_wr;
unsigned int sig_r_p;
unsigned int sig_r_u;
unsigned int sig_r_v;
unsigned int sig_r_x1;
unsigned int sig_ubus[8];
unsigned int sig_u_v;
unsigned int sig_vbus[8];
unsigned int sig__instr[8];
unsigned int sig_dbus[8];
unsigned int sig_regbit[16];

class Point
{
public:
   int x;
   int y;
};

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
   float ReadOutput();
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
   int x1, y1; // TL of bounding box
   int x2, y2; // BR of bounding box
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

float Signal::ReadOutput()
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

   // Record the min/max bounding box
   if (x < x1) {
      x1 = x;
   }
   if (y < y1) {
      y1 = y;
   }
   if (x > x2) {
      x2 = x;
   }
   if (y > y2) {
      y2 = y;
   }

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


// ===============================================================
// Netlist generation methods
// ===============================================================

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

vector<Point> walk_boundary(uint16_t *sigs, int start_x, int start_y, int min_x, int min_y, int max_x, int max_y, bool debug) {
   int sig = sigs[start_y * size_x + start_x];

   // Trace the boundary using the "square tracing" algorithm

   // Starting point is the top left corner, so start facing right
   vector<Point> boundary;

   int len_in_px = 0;
   int x         = start_x;
   int y         = start_y;
   int dir       = DIR_R;
   int last_x    = x; // The last point on the boundary that was visited
   int last_y    = y;


   // Output the start point, and mark as visited
   Point start;
   start.x = x;
   start.y = y;
   boundary.push_back(start);
   sigs[y * size_x + x] |= 0x8000;

   if (debug) {
      printf("walk_boundary: pushing start point %d,%d\n", start.x, start.y);
   }

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
      if (x >= min_x && x < max_x && y >= min_y && y < max_y) {
         point = sigs[y * size_x + x];
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

      // Ensure we process each boundary point just once
      if (point && point < 0x8000) {

         // Mark as visited
         sigs[y * size_x + x] |= 0x8000;

         // Test if move was diagonal, and if so also visit the corner point
         if (x != last_x && y != last_y) {
            Point w;
            if (x > last_x) {
               if (y > last_y) {
                  // case 1
                  w.x = last_x;
                  w.y = y;
               } else {
                  // case 2
                  w.x = x;
                  w.y = last_y;
               }
            } else {
               if (y > last_y) {
                  // case 3
                  w.x = x;
                  w.y = last_y;
               } else {
                  // case 4
                  w.x = last_x;
                  w.y = y;
               }
            }
            boundary.push_back(w);
            if (debug) {
               printf("walk_boundary: pushing extra point %d,%d\n", w.x, w.y);
            }


         }

         Point pt;
         pt.x = x;
         pt.y = y;
         boundary.push_back(pt);
         if (debug) {
            printf("walk_boundary: pushing point %d,%d\n", pt.x, pt.y);
         }

         last_x = x;
         last_y = y;

      }

      len_in_px++;

   } while (len_in_px < 100000 && (x != start_x || y != start_y));

   // Re-push the start point, as it makes the next stages easier
   boundary.push_back(start);

   return boundary;
}

vector<Point> remove_staircase(vector<Point> boundary, bool debug) {
   vector<Point> result;
   int i = 0;

   while (i < boundary.size()) {
      Point a = boundary[i];
      result.push_back(a);
      if (debug) {
         printf("remove_staircase: pushing point %d,%d\n", a.x, a.y);
      }
      // Measure the length of the staircased diagonal starting at this point
      int len = 0;

      if (i < boundary.size() - 4) {

         // Point b will be on a diagonal of length 2
         Point b = boundary[i + 4];

         // Check it's a possible diagonal of length >= 2
         if (abs(b.x - a.x) == 2 && abs(b.y - a.y) == 2) {

            // Establish the direction and then position of the offset points for an outer edge
            int dx;
            int dy;
            if (b.x > a.x) {
               if (b.y > a.y) {
                  // case 1: down/right
                  dx = 0;
                  dy = 1;
               } else {
                  // case 2: up/right
                  dx = 1;
                  dy = 0;
               }
            } else {
               if (b.y > a.y) {
                  // case 3: down/left
                  dx = -1;
                  dy = 0;
               } else {
                  // case 4: up/left
                  dx = 0;
                  dy = -1;
               }
            }

            // Now follow the diagonal
            Point c = a;
            for (int j = i + 1; j < boundary.size() - 1; j += 2) {
               // Check the next point is off the diagonal at the expected offset dx,dy
               Point d = boundary[j];
               if (d.x != c.x + dx || d.y != c.y + dy) {
                  break;
               }
               // Check the next point is back on the diaginal
               Point e = boundary[j + 1];
               if (abs(e.x - a.x) != len + 1 || abs(e.y - a.y) != len + 1) {
                  break;
               }
               // Keep track of the last point on the diagonal
               c = e;
               // And it's length
               len++;
            }
         }
      }

      // Replace diagonals of length 2 or more
      if (len >= 2) {
         if (debug) {
            printf("walk_boundary: found starcase of length %d\n", len);
         }
         i += len * 2;
      } else {
         i++;
      }

   }
   return result;
}

vector<Point> compress_edges(vector<Point> boundary, bool debug) {
   vector<Point> result;
   int i = 0;
   while (i < boundary.size()) {
      Point a = boundary[i];
      if (debug) {
         printf("compress_edges: pushing point %d,%d\n", a.x, a.y);
      }
      result.push_back(a);
      // Find the last point that has the same x or y as a
      int len = 0;
      int j = i + 1;
      while (j < boundary.size() && (boundary[j].x == a.x || boundary[j].y == a.y)) {
         j++;
         len++;
      }
      if (len > 1) {
         if (debug) {
            printf("compress_edges: found line of length %d\n", len);
         }
         i += len;
      } else {
         i++;
      }
   }
   return result;
}

void trace_boundary(FILE *segfile, int layer, uint16_t *sigs, int start_x, int start_y, int min_x, int min_y, int max_x, int max_y) {
   int sig = sigs[start_y * size_x + start_x];
   ::fprintf(segfile, "[ %d,'%c',%d", sig, (signals[sig].pullup ? '+' : '-'), layer);

   printf("tracing signal %d on layer %d starting at %d, %d\n", sig, layer, start_x, start_y);

   bool debug = false; // sig == 3273 && layer == 1;

   vector<Point> boundary = walk_boundary(sigs, start_x, start_y, min_x, min_y, max_x, max_y, debug);
   boundary = remove_staircase(boundary, debug);
   boundary = compress_edges(boundary, debug);

   for (int i = 0; i < boundary.size(); i++) {
      ::fprintf(segfile, ",%d,%d", boundary[i].x, size_y - boundary[i].y - 1);
   }

   ::fprintf(segfile, "],\n");

   if (debug) {
      ::exit(1);
   }
}

void write_layer_segments(FILE *segfile, int layer, uint16_t *sigs, int constraint) {
   // Segment the image into two halves, which is good enough to break up any very
   // large shapes that contain holes. This is a bit of a cludge, but simple boundary
   // detection algorithms don't handle holes.
   int block_size_x = 2350;
   int block_size_y = 5000;
   if (layer == 0) {
      // For the metal layer, break up a bit more, as there are smaller loops
      block_size_y = 1250;
   }
   for (int min_y = 0; min_y < size_y; min_y += block_size_y) {
      int max_y = min_y + block_size_y;
      if (max_y > size_y) {
         max_y = size_y;
      }
      for (int min_x = 0; min_x < size_x; min_x += block_size_x) {
         int max_x = min_x + block_size_x;
         if (max_x > size_x) {
            max_x = size_x;
         }
         for (int y = min_y; y < max_y; y++) {
            int last_sig = 0;
            for (int x = min_x; x < max_x; x++) {
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
                     trace_boundary(segfile, layer, sigs, x, y, min_x, min_y, max_x, max_y);
                  }
               }
               last_sig = sig;
            }
         }
      }
   }
   uint16_t *sp = sigs;
   for (int i = 0; i < size_x * size_y; i++) {
      *sp &= 0x7FFF;
      sp++;
   }
}


void write_segdefs_file(string filename) {
   FILE* segfile = ::fopen(filename.c_str(), "wb");
   ::fputs("var segdefs = [\n", segfile);
   write_layer_segments(segfile, 0, signals_metal, CONSTRAINT_NONE);
   write_layer_segments(segfile, 1, signals_diff,  CONSTRAINT_SWITCHED);
   write_layer_segments(segfile, 3, signals_diff,  CONSTRAINT_GND);
   write_layer_segments(segfile, 4, signals_diff,  CONSTRAINT_VCC);
   write_layer_segments(segfile, 5, signals_poly,  CONSTRAINT_NONE);
   ::fputs("]\n", segfile);
   ::fclose(segfile);
}

void update_pullup_status() {
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
}

void output_nodenames_warning(FILE *file) {
   ::fprintf(file, "// ***********************************************************\n");
   ::fprintf(file, "// * This file is automatically generated by Z80Simulator.   *\n");
   ::fprintf(file, "// * Please do not manually edit! Instead, find a transistor *\n");
   ::fprintf(file, "// * that uses the new signal and use FindTransistor(x,y) in *\n");
   ::fprintf(file, "// * Z80Simulator to cause that signal to be added.          *\n");
   ::fprintf(file, "// * This seems a pain, but it has two advantages:           *\n");
   ::fprintf(file, "// * - all signals are then available in Z80Simulator        *\n");
   ::fprintf(file, "// * - it avoids renumbering issues if/when the PNGs change  *\n");
   ::fprintf(file, "// ***********************************************************\n");
}

void output_nodenames(FILE *file, string padstr, string sepstr, bool perfectz80) {
   const char *pad = padstr.c_str();
   const char *sep = sepstr.c_str();
   ::fprintf(file, "%s// Pads\n", pad);
   ::fprintf(file, "%svss%s %d,\n", pad, sep, SIG_GND);
   ::fprintf(file, "%svcc%s %d,\n", pad, sep, SIG_VCC);
   ::fprintf(file, "%sclk%s %d,\n", pad, sep, PAD_CLK);
   ::fprintf(file, "%sab0%s %d,\n", pad, sep, PAD_A0);
   ::fprintf(file, "%sab1%s %d,\n", pad, sep, PAD_A1);
   ::fprintf(file, "%sab2%s %d,\n", pad, sep, PAD_A2);
   ::fprintf(file, "%sab3%s %d,\n", pad, sep, PAD_A3);
   ::fprintf(file, "%sab4%s %d,\n", pad, sep, PAD_A4);
   ::fprintf(file, "%sab5%s %d,\n", pad, sep, PAD_A5);
   ::fprintf(file, "%sab6%s %d,\n", pad, sep, PAD_A6);
   ::fprintf(file, "%sab7%s %d,\n", pad, sep, PAD_A7);
   ::fprintf(file, "%sab8%s %d,\n", pad, sep, PAD_A8);
   ::fprintf(file, "%sab9%s %d,\n", pad, sep, PAD_A9);
   ::fprintf(file, "%sab10%s %d,\n", pad, sep, PAD_A10);
   ::fprintf(file, "%sab11%s %d,\n", pad, sep, PAD_A11);
   ::fprintf(file, "%sab12%s %d,\n", pad, sep, PAD_A12);
   ::fprintf(file, "%sab13%s %d,\n", pad, sep, PAD_A13);
   ::fprintf(file, "%sab14%s %d,\n", pad, sep, PAD_A14);
   ::fprintf(file, "%sab15%s %d,\n", pad, sep, PAD_A15);
   ::fprintf(file, "%s_reset%s %d,\n", pad, sep, PAD__RESET);
   ::fprintf(file, "%s_wait%s %d,\n", pad, sep, PAD__WAIT);
   ::fprintf(file, "%swait%s %d,\n", pad, sep, PAD__WAIT);
   ::fprintf(file, "%s_int%s %d,\n", pad, sep, PAD__INT);
   if (!perfectz80) {
      ::fprintf(file, "%sint%s %d,\n", pad, sep, PAD__INT);
   }
   ::fprintf(file, "%s_irq%s %d,\n", pad, sep, PAD__INT);
   ::fprintf(file, "%sirq%s %d,\n", pad, sep, PAD__INT);
   ::fprintf(file, "%s_nmi%s %d,\n", pad, sep, PAD__NMI);
   ::fprintf(file, "%snmi%s %d,\n", pad, sep, PAD__NMI);
   ::fprintf(file, "%s_busrq%s %d,\n", pad, sep, PAD__BUSRQ);
   ::fprintf(file, "%sbusrq%s %d,\n", pad, sep, PAD__BUSRQ);
   ::fprintf(file, "%s_m1%s %d,\n", pad, sep, PAD__M1);
   ::fprintf(file, "%s_rd%s %d,\n", pad, sep, PAD__RD);
   ::fprintf(file, "%s_wr%s %d,\n", pad, sep, PAD__WR);
   ::fprintf(file, "%s_mreq%s %d,\n", pad, sep, PAD__MREQ);
   ::fprintf(file, "%s_iorq%s %d,\n", pad, sep, PAD__IORQ);
   ::fprintf(file, "%s_rfsh%s %d,\n", pad, sep, PAD__RFSH);
   ::fprintf(file, "%sdb0%s %d,\n", pad, sep, PAD_D0);
   ::fprintf(file, "%sdb1%s %d,\n", pad, sep, PAD_D1);
   ::fprintf(file, "%sdb2%s %d,\n", pad, sep, PAD_D2);
   ::fprintf(file, "%sdb3%s %d,\n", pad, sep, PAD_D3);
   ::fprintf(file, "%sdb4%s %d,\n", pad, sep, PAD_D4);
   ::fprintf(file, "%sdb5%s %d,\n", pad, sep, PAD_D5);
   ::fprintf(file, "%sdb6%s %d,\n", pad, sep, PAD_D6);
   ::fprintf(file, "%sdb7%s %d,\n", pad, sep, PAD_D7);
   ::fprintf(file, "%s_halt%s %d,\n", pad, sep, PAD__HALT);
   ::fprintf(file, "%s_busak%s %d,\n", pad, sep, PAD__BUSAK);

   ::fprintf(file, "%s// T-States\n", pad);
   ::fprintf(file, "%st1%s %d,\n", pad, sep,  transistors[sig_t1].gate);
   ::fprintf(file, "%st2%s %d,\n", pad, sep,  transistors[sig_t2].gate);
   ::fprintf(file, "%st3%s %d,\n", pad, sep,  transistors[sig_t3].gate);
   ::fprintf(file, "%st4%s %d,\n", pad, sep,  transistors[sig_t4].gate);
   ::fprintf(file, "%st5%s %d,\n", pad, sep,  transistors[sig_t5].gate);
   ::fprintf(file, "%st6%s %d,\n", pad, sep,  transistors[sig_t6].gate);

   ::fprintf(file, "%s// Machine cycles\n", pad);
   ::fprintf(file, "%sm1%s %d,\n", pad, sep,  transistors[sig_m1].gate);
   ::fprintf(file, "%sm2%s %d,\n", pad, sep,  transistors[sig_m2].gate);
   ::fprintf(file, "%sm3%s %d,\n", pad, sep,  transistors[sig_m3].gate);
   ::fprintf(file, "%sm4%s %d,\n", pad, sep,  transistors[sig_m4].gate);
   ::fprintf(file, "%sm5%s %d,\n", pad, sep,  transistors[sig_m5].gate);
   ::fprintf(file, "%sm6%s %d,\n", pad, sep,  transistors[sig_m6].gate);

   ::fprintf(file, "%s// EXX latches\n", pad);
   ::fprintf(file, "%sex_af%s %d,\n", pad, sep,  transistors[sig_ex_af].gate);
   ::fprintf(file, "%sex_bcdehl%s %d,\n", pad, sep,  transistors[sig_ex_bcdehl].gate);
   ::fprintf(file, "%sex_dehl0%s %d,\n", pad, sep,  transistors[sig_ex_dehl0].gate);
   ::fprintf(file, "%sex_dehl1%s %d,\n", pad, sep,  transistors[sig_ex_dehl1].gate);
   ::fprintf(file, "%sex_dehl_combined%s %d,\n", pad, sep,  transistors[sig_ex_dehl_combined].gate);

   ::fprintf(file, "%s// Registers\n", pad);
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_a%d%s %d,\n",     pad, i, sep, transistors[reg_a[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_f%d%s %d,\n",     pad, i, sep, transistors[reg_f[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_b%d%s %d,\n",     pad, i, sep, transistors[reg_b[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_c%d%s %d,\n",     pad, i, sep, transistors[reg_c[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_d%d%s %d,\n",     pad, i, sep, transistors[reg_d[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_e%d%s %d,\n",     pad, i, sep, transistors[reg_e[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_h%d%s %d,\n",     pad, i, sep, transistors[reg_h[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_l%d%s %d,\n",     pad, i, sep, transistors[reg_l[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_w%d%s %d,\n",     pad, i, sep, transistors[reg_w[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_z%d%s %d,\n",     pad, i, sep, transistors[reg_z[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_pch%d%s %d,\n",   pad, i, sep, transistors[reg_pch[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_pcl%d%s %d,\n",   pad, i, sep, transistors[reg_pcl[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_sph%d%s %d,\n",   pad, i, sep, transistors[reg_sph[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_spl%d%s %d,\n",   pad, i, sep, transistors[reg_spl[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_ixh%d%s %d,\n",   pad, i, sep, transistors[reg_ixh[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_ixl%d%s %d,\n",   pad, i, sep, transistors[reg_ixl[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_iyh%d%s %d,\n",   pad, i, sep, transistors[reg_iyh[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_iyl%d%s %d,\n",   pad, i, sep, transistors[reg_iyl[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_i%d%s %d,\n",     pad, i, sep, transistors[reg_i[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_r%d%s %d,\n",     pad, i, sep, transistors[reg_r[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_aa%d%s %d,\n",    pad, i, sep, transistors[reg_a2[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_ff%d%s %d,\n",    pad, i, sep, transistors[reg_f2[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_bb%d%s %d,\n",    pad, i, sep, transistors[reg_b2[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_cc%d%s %d,\n",    pad, i, sep, transistors[reg_c2[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_dd%d%s %d,\n",    pad, i, sep, transistors[reg_d2[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_ee%d%s %d,\n",    pad, i, sep, transistors[reg_e2[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_hh%d%s %d,\n",    pad, i, sep, transistors[reg_h2[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sreg_ll%d%s %d,\n",    pad, i, sep, transistors[reg_l2[i]].gate);
   }

   ::fprintf(file, "%s// Data buses and control\n", pad);
   ::fprintf(file, "%sdp_dl%s %d,\n", pad, sep,  transistors[sig_dp_dl].gate);
   ::fprintf(file, "%sdl_dp%s %d,\n", pad, sep,  transistors[sig_dl_dp].gate);
   ::fprintf(file, "%sload_ir%s %d,\n", pad, sep,  transistors[sig_load_ir].gate);
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sdlatch%d%s %d,\n",    pad, i, sep, transistors[sig_dlatch[i]].gate);
   }
   ::fprintf(file, "%sdl_d%s %d,\n", pad, sep,  transistors[sig_dl_d].gate);
   ::fprintf(file, "%sd_dl%s %d,\n", pad, sep,  transistors[sig_d_dl].gate);
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sdbus%d%s %d,\n",      pad, i, sep, transistors[sig_dbus[i]].source);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%s_instr%d%s %d,\n",    pad, i, sep, transistors[sig__instr[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%sinstr%d%s %d,\n",     pad, i, sep, transistors[sig_instr[i]].gate);
   }
   ::fprintf(file, "%sd_u%s %d,\n", pad, sep,  transistors[sig_d_u].gate);
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%subus%d%s %d,\n",      pad, i, sep, transistors[sig_ubus[i]].gate);
   }
   ::fprintf(file, "%su_v%s %d,\n", pad, sep,  transistors[sig_u_v].gate);
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%svbus%d%s %d,\n",      pad, i, sep, transistors[sig_vbus[i]].gate);
   }
   ::fprintf(file, "%srl_wr%s %d,\n", pad, sep,  transistors[sig_rl_wr].gate);
   ::fprintf(file, "%srh_wr%s %d,\n", pad, sep,  transistors[sig_rh_wr].gate);
   ::fprintf(file, "%sr_u%s %d,\n", pad, sep,  transistors[sig_r_u].gate);
   ::fprintf(file, "%sr_v%s %d,\n", pad, sep,  transistors[sig_r_v].gate);
   for (int i = 0; i < 16; i++) {
      ::fprintf(file, "%sregbit%d%s %d,\n", pad, i, sep,  transistors[sig_regbit[i]].source);
   }
   ::fprintf(file, "%sr_p%s %d,\n", pad, sep,  transistors[sig_r_p].gate);
   ::fprintf(file, "%sr_x1%s %d,\n", pad, sep,  transistors[sig_r_x1].gate);
   for (int i = 0; i < 16; i++) {
      ::fprintf(file, "%spcbit%d%s %d,\n", pad, i, sep,  transistors[sig_pcbit[i]].gate);
   }

   ::fprintf(file, "%s// ALU\n", pad);
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%salubus%d%s %d,\n",    pad, i, sep, transistors[sig_alubus[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%salua%d%s %d,\n",      pad, i, sep, transistors[sig_alua[i]].gate);
   }
   for (int i = 0; i < 8; i++) {
      ::fprintf(file, "%salub%d%s %d,\n",      pad, i, sep, transistors[sig_alub[i]].gate);
   }
   for (int i = 0; i < 4; i++) {
      ::fprintf(file, "%saluout%d%s %d,\n", pad, i, sep,  transistors[sig_aluout[i]].gate);
   }
   for (int i = 0; i < 4; i++) {
      ::fprintf(file, "%salulat%d%s %d,\n", pad, i, sep,  transistors[sig_alulat[i]].gate);
   }

   ::fprintf(file, "%s// PLA\n", pad);
   for (int i = 0; i < 99; i++) {
      if (i == 83) {
         ::fprintf(file, "%spla%d%s %d,\n", pad, i, sep,  transistors[sig_pla[i]].source);
      } else {
         ::fprintf(file, "%spla%d%s %d,\n", pad, i, sep,  transistors[sig_pla[i]].gate);
      }
   }
}


void write_nodenames_file(string filename) {
   FILE* nodefile = ::fopen(filename.c_str(), "wb");
   output_nodenames_warning(nodefile);
   ::fputs("var nodenames ={\n", nodefile);
   output_nodenames(nodefile, "", ":", false);
   ::fputs("}\n", nodefile);
   ::fclose(nodefile);
}


void check_transistor_spacing() {
   for (unsigned int i = 0; i < transistors.size(); i++) {

      Transistor ti = transistors[i];

      int count1 = 0;
      int count2 = 0;
      for (unsigned int j = 0; j < ti.drainconnections.size(); j++) {
         if (ti.drainconnections[j].index != i) {
            count1++;
            if (ti.drainconnections[j].terminal != GATE) {
               count2++;
            }
         }
      }

      // Look for one or two connections to other sources/drains
      if (ti.source == SIG_GND && (count1 == 1 || count1 == 2) && count1 == count2) {

         int minx = 1000000;
         int miny = 1000000;


         for (unsigned int j = 0; j < ti.drainconnections.size(); j++)
         {
            if (ti.drainconnections[j].index == i) {
               continue;
            }

            Transistor tj = transistors[ti.drainconnections[j].index];

            // Do the rectangles overlap in the X dimension
            if ((ti.x1 >= tj.x1 && ti.x1 <= tj.x2) || (ti.x2 >= tj.x1 && ti.x2 <= tj.x2)) {
               int dy1 = ti.y1 - tj.y2;
               int dy2 = ti.y2 - tj.y1;
               if (dy1 < 0) {
                  dy1 = -dy1;
               }
               if (dy2 < 0) {
                  dy2 = -dy2;
               }
               if (dy1 < miny) {
                  miny = dy1;
               }
               if (dy2 < miny) {
                  miny = dy2;
               }
            }

            // Do the rectangles overlap in the Y dimension
            if ((ti.y1 >= tj.y1 && ti.y1 <= tj.y2) || (ti.y2 >= tj.y1 && ti.y2 <= tj.y2)) {
               int dx1 = ti.x1 - tj.x2;
               int dx2 = ti.x2 - tj.x1;
               if (dx1 < 0) {
                  dx1 = -dx1;
               }
               if (dx2 < 0) {
                  dx2 = -dx2;
               }
               if (dx1 < minx) {
                  minx = dx1;
               }
               if (dx2 < minx) {
                  minx = dx2;
               }
            }
         }
         int sx = ti.x2 - ti.x1 + 1;
         int sy = ti.y2 - ti.y1 + 1;
         int min;
         if (sx > sy) {
            min = miny;
         } else if (sy > sx) {
            min = minx;
         } else {
            min = (minx < miny) ? minx : miny;
         }
         printf("t%4d (%4d, %4d, %4d, %4d) %d %d spacing min %d\n", i, ti.x1, ti.y1, sx, sy, count1, count2, min);
      }
   }
}

void write_transdefs_file(string filename) {
   // The format here is
   //   name
   //   gate,c1,c2
   //   bb (bounding box: xmin, xmax, ymin, ymax)
   //   geometry (unused) (width1, width2, length, #segments, area)
   //   weak (boolean) (marks weak transistors, whether pullups or pass gates)
   //

   // Maintain a list of signals that should be forces to ground sue to traps
   // (the depletion mode transistors are processed first, so two passes are not required)

   vector<int> force_to_ground;

   FILE* trfile = ::fopen(filename.c_str(), "wb");
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
         int gate   = t.gate;
         int source = t.source;
         int drain  = t.drain;
         if (t.depletion) {
            printf("Warning: t%d is depletion mode (g=%d, s=%d, d=%d) but not a pullup; trap? replacing signal %d with ground in netlist\n", i, t.gate, t.source, t.drain, t.drain);
            // All the traps have a grounded source
            force_to_ground.push_back(t.drain);
            gate = SIG_GND;
            drain = SIG_GND;
            source = SIG_GND;
         }
         for (int j = 0; j < force_to_ground.size(); j++) {
            int trap = force_to_ground[j];
            if (gate == trap ) {
               gate = SIG_GND;
               printf("Forcing gate to ground on transistor %d due to trap\n", i);
            }
            if (source == trap) {
               source = SIG_GND;
               printf("Forcing source to ground on transistor %d due to trap\n", i);
            }
            if (drain == trap) {
               drain = SIG_GND;
               printf("Forcing drain to ground on transistor %d due to trap\n", i);
            }
         }
         ::fprintf(trfile, "['t%d',%d,%d,%d,", i, gate, source, drain);
         ::fprintf(trfile, "[%d,%d,%d,%d],", t.x1, (t.x2 + 1), (size_y - 1) - (t.y2 + 1), (size_y - 1) - t.y1);
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


void write_perfect_z80_file(string filename) {
   // Maintain a list of signals that should be forces to ground sue to traps
   // (the depletion mode transistors are processed first, so two passes are not required)

   vector<int> force_to_ground;

   FILE* trfile = ::fopen(filename.c_str(), "wb");
   output_nodenames_warning(trfile);
   ::fputs("#include \"types.h\"\n", trfile);
   ::fputs("\n", trfile);
   ::fputs("enum {\n", trfile);
   output_nodenames(trfile, "   ", " =", true);
   ::fputs("};\n", trfile);
   ::fputs("\n", trfile);
   ::fputs("BOOL\n", trfile);
   ::fputs("netlist_z80_node_is_pullup[] = {\n", trfile);
   int num_per_line = 23;
   for (int i = 0; i < signals.size(); i++) {
      if ((i % num_per_line) == 0) {
         ::fputs("  ", trfile);
      }
      if (signals[i].pullup) {
         ::fputs(" 1", trfile);
      } else {
         ::fputs(" 0", trfile);
      }
      if (i != signals.size() - 1) {
         ::fputs(",", trfile);
      }
      if ((i % num_per_line) == (num_per_line - 1)) {
         ::fputs("\n", trfile);
      }
   }
   ::fputs("\n", trfile);
   ::fputs("};\n", trfile);
   ::fputs("\n", trfile);
   ::fputs("netlist_transdefs\n", trfile);
   ::fputs("netlist_z80_transdefs[] = {\n", trfile);
   for (int i = 0; i < transistors.size(); i++) {
      Transistor t = transistors[i];
      int gate   = t.gate;
      int source = t.source;
      int drain  = t.drain;
      if (t.source == SIG_GND && t.gate == SIG_GND) {
         printf("Warning: t%d is a protection diode (g=%d, s=%d, d=%d) - excluding\n", i, t.gate, t.source, t.drain);
         gate = SIG_GND;
         source = SIG_GND;
         drain = SIG_GND;
      }
      if (t.source == 0) {
         printf("Warning: t%d has disconnected source (area = %d) - excluding\n", i, (int) t.area);
         gate = SIG_GND;
         source = SIG_GND;
         drain = SIG_GND;
      }
      if (t.drain == 0) {
         printf("Warning: t%d has disconnected drain (area = %d) - excluding\n", i, (int) t.area);
         gate = SIG_GND;
         source = SIG_GND;
         drain = SIG_GND;
      }
      if (t.gate == 0) {
         printf("Warning: t%d has disconnected gate (area = %d) - excluding\n", i, (int) t.area);
         gate = SIG_GND;
         source = SIG_GND;
         drain = SIG_GND;
      }
      if ((t.source != t.gate || t.drain != SIG_VCC) && t.depletion) {
         printf("Warning: t%d is depletion mode (g=%d, s=%d, d=%d) but not a pullup; trap? replacing signal %d with ground in netlist\n", i, t.gate, t.source, t.drain, t.drain);
         // All the traps have a grounded source
         force_to_ground.push_back(t.drain);
         gate = SIG_GND;
         source = SIG_GND;
         drain = SIG_GND;
      }
      for (int j = 0; j < force_to_ground.size(); j++) {
         int trap = force_to_ground[j];
         if (gate == trap ) {
            gate = SIG_GND;
            printf("Forcing gate to ground on transistor %d due to trap\n", i);
         }
         if (source == trap) {
            source = SIG_GND;
            printf("Forcing source to ground on transistor %d due to trap\n", i);
         }
         if (drain == trap) {
            drain = SIG_GND;
            printf("Forcing drain to ground on transistor %d due to trap\n", i);
         }
      }
      ::fprintf(trfile, "   {%d, %d, %d}", gate, source, drain);
      if (i != transistors.size() - 1) {
         ::fputs(",\n", trfile);
      } else {
         ::fputs("\n", trfile);
      }
   }
   ::fputs("};\n", trfile);
   ::fclose(trfile);
}

// Everything starts here
int main(int argc, char *argv[])
{
   int64_t duration = GetTickCount();

   ZeroMemory(&memory[0], 65536 * sizeof(uint8_t));
   ZeroMemory(&ports[0], 256 * sizeof(uint8_t));

   // Simulated Z80 program

   int mem_addr = 0;

   memory[mem_addr++] = 0x31;
   memory[mem_addr++] = 0x80;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0xed;
   memory[mem_addr++] = 0x56;
   memory[mem_addr++] = 0x3e;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0xed;
   memory[mem_addr++] = 0x47;
   memory[mem_addr++] = 0xfb;
   memory[mem_addr++] = 0xfb;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0xed;
   memory[mem_addr++] = 0x57;
   memory[mem_addr++] = 0x18;
   memory[mem_addr++] = 0xfe;


   memory[0x0038] = 0xfb;
   memory[0x0039] = 0xc9;

   memory[0x0066] = 0xed;
   memory[0x0067] = 0x45;

#if 0

   // 0x00,                    // NOP
   // 0x31, 0x00, 0x01,        // LD SP,0x0100
   // 0xaf,                    // XOR A
   // 0xee, 0xff,              // XOR A, 0xFF
   // 0x3e, 0x00,              // LD A, 0x00
   // 0x00,                    // NOP
   // 0x37,                    // SCF
   // 0x00,                    // NOP
   // 0x00,                    // NOP
   // 0xf5,                    // PUSH AF
   // 0xe1,                    // POP HL
   // 0x00,                    // NOP
   // 0x76,                    // HALT
   // 0x00,                    // NOP

   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x31;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x01;
   memory[mem_addr++] = 0xaf;
   memory[mem_addr++] = 0xee;
   memory[mem_addr++] = 0xff;
   memory[mem_addr++] = 0x3e;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x37;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0xf5;
   memory[mem_addr++] = 0xe1;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x76;
   memory[mem_addr++] = 0x00;
   // 0x00,                    // NOP
   // 0x31, 0x00, 0x01,        // LD SP,0x0100
   // 0xaf,                    // XOR A
   // 0x3e, 0xff,              // LD A, 0xFF
   // 0x00,                    // NOP
   // 0x37,                    // SCF
   // 0x00,                    // NOP
   // 0x00,                    // NOP
   // 0xf5,                    // PUSH AF
   // 0xe1,                    // POP HL
   // 0x00,                    // NOP
   // 0x76,                    // HALT
   // 0x00,                    // NOP

   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x31;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x01;
   memory[mem_addr++] = 0xaf;
   memory[mem_addr++] = 0x3e;
   memory[mem_addr++] = 0xff;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x37;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0xf5;
   memory[mem_addr++] = 0xe1;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x76;
   memory[mem_addr++] = 0x00;

   // 0x00,                    // NOP
   // 0x31, 0x00, 0x01,        // LD SP,0x0100
   // 0xCD, 0x0B, 0x00,        // CALL $000B
   // 0x00,                    // NOP
   // 0x21, 0x78, 0x56,        // LD HL,$5678
   // 0x21, 0x34, 0x12,        // LD HL,$1234
   // 0xe5,                    // PUSH HL
   // 0x00,                    // NOP
   // 0x00,                    // NOP

   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x31;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x01;
   memory[mem_addr++] = 0xcd;
   memory[mem_addr++] = 0x0b;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x21;
   memory[mem_addr++] = 0x78;
   memory[mem_addr++] = 0x56;
   memory[mem_addr++] = 0x21;
   memory[mem_addr++] = 0x34;
   memory[mem_addr++] = 0x12;
   memory[mem_addr++] = 0xe5;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x00;

   // 0x3C,                    // INC A
   // 0x04,                    // INC B
   // 0x15,                    // DEC D
   // 0x24,                    // INC H
   // 0xEB,                    // EXX DE,HL
   // 0x00,                    // NOP
   // 0x3C,                    // INC A
   // 0x04,                    // INC B
   // 0x15,                    // DEC D
   // 0x24,                    // INC H
   // 0xD9,                    // EXX
   // 0x00,                    // NOP
   // 0x3C,                    // INC A
   // 0x04,                    // INC B
   // 0x15,                    // DEC D
   // 0x24,                    // INC H
   // 0xEB,                    // EXX DE,HL
   // 0x00,                    // NOP
   // 0x3C,                    // INC A
   // 0x04,                    // INC B
   // 0x15,                    // DEC D
   // 0x24,                    // INC H
   // 0x08,                    // EXX AF,AF'
   // 0x00,                    // NOP
   // 0x3C,                    // INC A
   // 0x04,                    // INC B
   // 0x15,                    // DEC D
   // 0x24,                    // INC H
   // 0x00,                    // NOP
   // 0x00,                    // NOP
   // 0x00,                    // NOP

   memory[mem_addr++] = 0x3C;
   memory[mem_addr++] = 0x04;
   memory[mem_addr++] = 0x15;
   memory[mem_addr++] = 0x24;
   memory[mem_addr++] = 0xEB;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x3C;
   memory[mem_addr++] = 0x04;
   memory[mem_addr++] = 0x15;
   memory[mem_addr++] = 0x24;
   memory[mem_addr++] = 0xD9;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x3C;
   memory[mem_addr++] = 0x04;
   memory[mem_addr++] = 0x15;
   memory[mem_addr++] = 0x24;
   memory[mem_addr++] = 0xEB;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x3C;
   memory[mem_addr++] = 0x04;
   memory[mem_addr++] = 0x15;
   memory[mem_addr++] = 0x24;
   memory[mem_addr++] = 0x08;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x3C;
   memory[mem_addr++] = 0x04;
   memory[mem_addr++] = 0x15;
   memory[mem_addr++] = 0x24;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x00;

   // 0x21, 0x00, 0x01,        // LD HL,$0100
   // 0x36, 0xCC,              // LD (HL),$CC
   // 0x00,                    // NOP
   // 0x7E,                    // LD A, (HL)
   // 0x00,                    // NOP

   memory[mem_addr++] = 0x21;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x01;
   memory[mem_addr++] = 0x36;
   memory[mem_addr++] = 0xcc;
   memory[mem_addr++] = 0x00;
   memory[mem_addr++] = 0x7e;
   memory[mem_addr++] = 0x00;

   // Pavel's original test program
   // 0x21, 0x34, 0x12,        // LD HL,$1234
   // 0x31, 0xfe, 0xdc,        // LD SP,0xDCFE
   // 0xe5,                    // PUSH HL
   // 0x21, 0x78, 0x56,        // LD HL,$5678
   // 0xe3,                    // EX (SP),HL
   // 0xdd, 0x21, 0xbc,0x9a,   // LD IX, 0x9ABC
   // 0xdd, 0xe3,              // EX (SP),IX
   // 0x76,                    // HALT
   // 0x00                     // NOP

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
   memory[mem_addr++] = 0x00;

#endif

   printf("Test program contains %d bytes\n", mem_addr);

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
            // min/max bounding box
            x1 = size_x - 1;
            y1 = size_y - 1;
            x2 = 0;
            y2 = 0;
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
            pomtran.x1 = x1;
            pomtran.y1 = y1;
            pomtran.x2 = x2;
            pomtran.y2 = y2;
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
   bool pom_m1 = true;
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

   sig_t1 = FindTransistor(572, 1203);
   sig_t2 = FindTransistor(831, 1895);
   sig_t3 = FindTransistor(901, 1242);
   sig_t4 = FindTransistor(876, 1259);
   sig_t5 = FindTransistor(825, 1958);
   sig_t6 = FindTransistor(921, 1211);

   sig_m1 = FindTransistor(1051, 1057);
   sig_m2 = FindTransistor(1014, 1165);
   sig_m3 = FindTransistor(1018, 1319);
   sig_m4 = FindTransistor(1027, 1300);
   sig_m5 = FindTransistor(1014, 1243);
   sig_m6 = FindTransistor(3180, 3026);

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

   //sig_ex_af            = FindTransistor(2306, 3051);
   //sig_ex_bcdehl        = FindTransistor(979, 3093);
   //sig_ex_dehl0         = FindTransistor(1449, 2971);
   //sig_ex_dehl1         = FindTransistor(1193, 2981);
   //sig_ex_dehl_combined = FindTransistor(1443, 3001);

   sig_iff1             = FindTransistor(3722,826); // t2712
   sig_iff2             = FindTransistor(3592,938); // t3187

   sig_ex_af            = FindTransistor(2455, 3030);
   sig_ex_bcdehl        = FindTransistor( 661, 3130);
   sig_ex_dehl0         = FindTransistor(1018, 2981);
   sig_ex_dehl1         = FindTransistor(1193, 2981);
   sig_ex_dehl_combined = FindTransistor(1443, 3001);

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

    sig_alua[0] = FindTransistor(4029,3771);
    sig_alua[1] = FindTransistor(4029,3974);
    sig_alua[2] = FindTransistor(4029,4177);
    sig_alua[3] = FindTransistor(4029,4381);
    sig_alua[4] = FindTransistor(4034,3801);
    sig_alua[5] = FindTransistor(4034,4005);
    sig_alua[6] = FindTransistor(4034,4209);
    sig_alua[7] = FindTransistor(4034,4412);
    sig_alubus[0] = FindTransistor(3003,3712);
    sig_alubus[1] = FindTransistor(3003,3915);
    sig_alubus[2] = FindTransistor(3003,4118);
    sig_alubus[3] = FindTransistor(3003,4321);
    sig_alubus[4] = FindTransistor(3009,3842);
    sig_alubus[5] = FindTransistor(3009,4045);
    sig_alubus[6] = FindTransistor(3009,4249);
    sig_alubus[7] = FindTransistor(3009,4449);
    sig_alub[0] = FindTransistor(3092,3770);
    sig_alub[1] = FindTransistor(3092,3973);
    sig_alub[2] = FindTransistor(3092,4177);
    sig_alub[3] = FindTransistor(3092,4380);
    sig_alub[4] = FindTransistor(3104,3803);
    sig_alub[5] = FindTransistor(3104,4006);
    sig_alub[6] = FindTransistor(3104,4210);
    sig_alub[7] = FindTransistor(3104,4413);
    sig_alulat[0] = FindTransistor(3771,3771);
    sig_alulat[1] = FindTransistor(3771,3974);
    sig_alulat[2] = FindTransistor(3771,4178);
    sig_alulat[3] = FindTransistor(3771,4381);
    sig_aluout[0] = FindTransistor(3779,3739);
    sig_aluout[1] = FindTransistor(3779,3942);
    sig_aluout[2] = FindTransistor(3779,4146);
    sig_aluout[3] = FindTransistor(3611,3349);
    sig_dlatch[0] = FindTransistor(4245,555);
    sig_dlatch[1] = FindTransistor(4368,998);
    sig_dlatch[2] = FindTransistor(4256,1828);
    sig_dlatch[3] = FindTransistor(4260,4093);
    sig_dlatch[4] = FindTransistor(4256,4407);
    sig_dlatch[5] = FindTransistor(4282,3807);
    sig_dlatch[6] = FindTransistor(4388,3415);
    sig_dlatch[7] = FindTransistor(4368,1324);
    sig_dl_dp = FindTransistor(4274,1998);
    sig_dl_d = FindTransistor(4476,2311);
    sig_dp_dl = FindTransistor(4467,2293);
    sig_d_dl = FindTransistor(4355,1047);
    sig_d_u = FindTransistor(4253,3488);
    sig_instr[0] = FindTransistor(4097,1224);
    sig_instr[1] = FindTransistor(4097,1276);
    sig_instr[2] = FindTransistor(4097,1329);
    sig_instr[3] = FindTransistor(4097,1485);
    sig_instr[4] = FindTransistor(4097,1538);
    sig_instr[5] = FindTransistor(4097,1590);
    sig_instr[6] = FindTransistor(4097,1381);
    sig_instr[7] = FindTransistor(4097,1433);
    sig_load_ir = FindTransistor(4255,1393);
    sig_pcbit[0] = FindTransistor(1149,3223);
    sig_pcbit[1] = FindTransistor(1143,3324);
    sig_pcbit[2] = FindTransistor(1149,3367);
    sig_pcbit[3] = FindTransistor(1143,3468);
    sig_pcbit[4] = FindTransistor(1149,3511);
    sig_pcbit[5] = FindTransistor(1143,3612);
    sig_pcbit[6] = FindTransistor(1149,3655);
    sig_pcbit[7] = FindTransistor(1143,3756);
    sig_pcbit[8] = FindTransistor(1149,3819);
    sig_pcbit[9] = FindTransistor(1143,3920);
    sig_pcbit[10] = FindTransistor(1149,3963);
    sig_pcbit[11] = FindTransistor(1143,4064);
    sig_pcbit[12] = FindTransistor(1149,4107);
    sig_pcbit[13] = FindTransistor(1143,4208);
    sig_pcbit[14] = FindTransistor(1149,4251);
    sig_pcbit[15] = FindTransistor(1143,4352);
    sig_pla[0] = FindTransistor(1435,947);
    sig_pla[1] = FindTransistor(852,2686);
    sig_pla[2] = FindTransistor(935,2680);
    sig_pla[3] = FindTransistor(990,2664);
    sig_pla[4] = FindTransistor(1080,2392);
    sig_pla[5] = FindTransistor(1204,2382);
    sig_pla[6] = FindTransistor(1554,1673);
    sig_pla[7] = FindTransistor(1459,2062);
    sig_pla[8] = FindTransistor(1305,2410);
    sig_pla[9] = FindTransistor(1323,2410);
    sig_pla[10] = FindTransistor(1400,2410);
    sig_pla[11] = FindTransistor(1586,2020);
    sig_pla[12] = FindTransistor(1607,2020);
    sig_pla[13] = FindTransistor(1693,1812);
    sig_pla[14] = FindTransistor(1710,1891);
    sig_pla[15] = FindTransistor(2282,2129);
    sig_pla[16] = FindTransistor(1800,1640);
    sig_pla[17] = FindTransistor(1744,2397);
    sig_pla[18] = FindTransistor(1761,2397);
    sig_pla[19] = FindTransistor(1852,1731);
    sig_pla[20] = FindTransistor(1901,2397);
    sig_pla[21] = FindTransistor(1651,2263);
    sig_pla[22] = FindTransistor(2011,1103);
    sig_pla[23] = FindTransistor(1917,2074);
    sig_pla[24] = FindTransistor(2083,1678);
    sig_pla[25] = FindTransistor(2022,2412);
    sig_pla[26] = FindTransistor(2131,1641);
    sig_pla[27] = FindTransistor(2083,2445);
    sig_pla[28] = FindTransistor(2203,1815);
    sig_pla[29] = FindTransistor(2225,1660);
    sig_pla[30] = FindTransistor(2241,1660);
    sig_pla[31] = FindTransistor(2263,1660);
    sig_pla[32] = FindTransistor(2165,2539);
    sig_pla[33] = FindTransistor(2343,1833);
    sig_pla[34] = FindTransistor(2359,1833);
    sig_pla[35] = FindTransistor(2381,1678);
    sig_pla[36] = FindTransistor(2406,1776);
    sig_pla[37] = FindTransistor(2343,1833);
    sig_pla[38] = FindTransistor(2453,1660);
    sig_pla[39] = FindTransistor(2403,2343);
    sig_pla[40] = FindTransistor(2498,1852);
    sig_pla[41] = FindTransistor(2447,1096);
    sig_pla[42] = FindTransistor(2531,1678);
    sig_pla[43] = FindTransistor(2554,1660);
    sig_pla[44] = FindTransistor(2468,1089);
    sig_pla[45] = FindTransistor(2667,2515);
    sig_pla[46] = FindTransistor(3544,883);
    sig_pla[47] = FindTransistor(2623,1678);
    sig_pla[48] = FindTransistor(2643,2111);
    sig_pla[49] = FindTransistor(2668,1701);
    sig_pla[50] = FindTransistor(2694,1776);
    sig_pla[51] = FindTransistor(2771,1080);
    sig_pla[52] = FindTransistor(2805,1817);
    sig_pla[53] = FindTransistor(2824,1702);
    sig_pla[54] = FindTransistor(2869,1702);
    sig_pla[55] = FindTransistor(2883,1849);
    sig_pla[56] = FindTransistor(2962,846);
    sig_pla[57] = FindTransistor(2788,2335);
    sig_pla[58] = FindTransistor(2961,1817);
    sig_pla[59] = FindTransistor(2979,1815);
    sig_pla[60] = FindTransistor(2875,2307);
    sig_pla[61] = FindTransistor(3084,1658);
    sig_pla[62] = FindTransistor(3077,2704);
    sig_pla[63] = FindTransistor(3196,856);
    sig_pla[64] = FindTransistor(3209,1101);
    sig_pla[65] = FindTransistor(3220,1108);
    sig_pla[66] = FindTransistor(3240,1680);
    sig_pla[67] = FindTransistor(3260,1658);
    sig_pla[68] = FindTransistor(3289,2023);
    sig_pla[69] = FindTransistor(3300,2023);
    sig_pla[70] = FindTransistor(3321,2023);
    sig_pla[71] = FindTransistor(3339,2023);
    sig_pla[72] = FindTransistor(3372,1680);
    sig_pla[73] = FindTransistor(3389,1774);
    sig_pla[74] = FindTransistor(3419,1853);
    sig_pla[75] = FindTransistor(3433,1774);
    sig_pla[76] = FindTransistor(3444,2013);
    sig_pla[77] = FindTransistor(3136,2170);
    sig_pla[78] = FindTransistor(3520,1738);
    sig_pla[79] = FindTransistor(3543,1738);
    sig_pla[80] = FindTransistor(3566,1738);
    sig_pla[81] = FindTransistor(3589,1640);
    sig_pla[82] = FindTransistor(3615,1640);
    sig_pla[84] = FindTransistor(3661,1738);
    sig_pla[85] = FindTransistor(3691,1815);
    sig_pla[86] = FindTransistor(3714,1738);
    sig_pla[87] = FindTransistor(3729,1056);
    sig_pla[88] = FindTransistor(3766,1738);
    sig_pla[89] = FindTransistor(3788,1681);
    sig_pla[90] = FindTransistor(3491,2349);
    sig_pla[91] = FindTransistor(3894,2024);
    sig_pla[92] = FindTransistor(3845,1681);
    sig_pla[93] = FindTransistor(3524,2311);
    sig_pla[94] = FindTransistor(3883,1681);
    sig_pla[95] = FindTransistor(2953,1199);
    sig_pla[96] = FindTransistor(3485,840);
    sig_pla[97] = FindTransistor(3757,890);
    sig_pla[98] = FindTransistor(4041,2331);
    sig_rh_wr = FindTransistor(2365,3952);
    sig_rl_wr = FindTransistor(2365,3464);
    sig_r_p = FindTransistor(1506,3953);
    sig_r_u = FindTransistor(2541,3252);
    sig_r_v = FindTransistor(2541,3848);
    sig_r_x1 = FindTransistor(1538,3547);
    sig_ubus[0] = FindTransistor(2845,3175);
    sig_ubus[1] = FindTransistor(2486,3344);
    sig_ubus[2] = FindTransistor(2790,3175);
    sig_ubus[3] = FindTransistor(2486,3488);
    sig_ubus[4] = FindTransistor(2493,3501);
    sig_ubus[5] = FindTransistor(2486,3632);
    sig_ubus[6] = FindTransistor(2745,3175);
    sig_ubus[7] = FindTransistor(2799,3175);
    sig_u_v = FindTransistor(2617,2920);
    sig_vbus[0] = FindTransistor(2493,3810);
    sig_vbus[1] = FindTransistor(2486,3940);
    sig_vbus[2] = FindTransistor(2493,3954);
    sig_vbus[3] = FindTransistor(2486,4084);
    sig_vbus[4] = FindTransistor(2493,4098);
    sig_vbus[5] = FindTransistor(2486,4228);
    sig_vbus[6] = FindTransistor(2493,4242);
    sig_vbus[7] = FindTransistor(2486,4372);
    sig__instr[0] = FindTransistor(4185,1222);
    sig__instr[1] = FindTransistor(4185,1274);
    sig__instr[2] = FindTransistor(4185,1326);
    sig__instr[3] = FindTransistor(4185,1483);
    sig__instr[4] = FindTransistor(4185,1535);
    sig__instr[5] = FindTransistor(4185,1587);
    sig__instr[6] = FindTransistor(4185,1378);
    sig__instr[7] = FindTransistor(4185,1431);
    sig_dbus[0] = FindTransistor(4300,2358);
    sig_dbus[1] = FindTransistor(4333,2349);
    sig_dbus[2] = FindTransistor(4350,2349);
    sig_dbus[3] = FindTransistor(4270,2287);
    sig_dbus[4] = FindTransistor(4299,2288);
    sig_dbus[5] = FindTransistor(4317,2288);
    sig_dbus[6] = FindTransistor(4236,2287);
    sig_dbus[7] = FindTransistor(4379,2348);
    sig_pla[83] = FindTransistor(3603,1856);
    sig_regbit[0] = FindTransistor(1575,3212);
    sig_regbit[1] = FindTransistor(2365,3320);
    sig_regbit[2] = FindTransistor(1575,3356);
    sig_regbit[3] = FindTransistor(2365,3464);
    sig_regbit[4] = FindTransistor(1575,3500);
    sig_regbit[5] = FindTransistor(2365,3608);
    sig_regbit[6] = FindTransistor(1575,3644);
    sig_regbit[7] = FindTransistor(2365,3752);
    sig_regbit[8] = FindTransistor(1575,3808);
    sig_regbit[9] = FindTransistor(1575,3935);
    sig_regbit[10] = FindTransistor(1575,3952);
    sig_regbit[11] = FindTransistor(2365,4060);
    sig_regbit[12] = FindTransistor(1575,4096);
    sig_regbit[13] = FindTransistor(2365,4204);
    sig_regbit[14] = FindTransistor(1575,4240);
    sig_regbit[15] = FindTransistor(2365,4348);
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

   check_transistor_spacing();

   // Annotate the transitors with their pullup status
   update_pullup_status();

   // Output the file for perfect z80 simulator
   write_perfect_z80_file("netlist_z80.h");

   // Output the files for the Visual Z80 simulator (chipsim)
   write_transdefs_file("transdefs.js");
   write_segdefs_file("segdefs.js");
   write_nodenames_file("nodenames.js");

   // =============================
   // End of saving colored bitmaps
   // =============================

   delete pombuf;
   delete signals_diff;
   delete signals_poly;
   delete signals_metal;

   // ::exit(1);

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
               if (i < DIVISOR * (100 + 200))
               {
                  pads[j].SetInputSignal(SIG_VCC);
               }
               else
               {
                  pads[j].SetInputSignal(SIG_GND);
               }
            }
            else if (pads[j].origsignal == PAD__NMI)
            {
               if (i < DIVISOR * (100 + 84))
               {
                  pads[j].SetInputSignal(SIG_VCC);
               }
               else
               {
                  pads[j].SetInputSignal(SIG_GND);
               }
            }
            else if (pads[j].origsignal == PAD__BUSRQ)
            {
                  pads[j].SetInputSignal(SIG_VCC);
            }
         }
         else if (pads[j].type == PAD_BIDIRECTIONAL) // we have to pull data bus up or down when memory, I/O or interrupt instruction is read
         {
            {
               if (!pom_m1 && !pom_iorq)
               {
                     if (
                        pads[j].origsignal == PAD_D7 ||
                        pads[j].origsignal == PAD_D6 ||
                        pads[j].origsignal == PAD_D5 ||
                        pads[j].origsignal == PAD_D3 ||
                        pads[j].origsignal == PAD_D0
                        ) {
                        pads[j].SetInputSignal(SIG_VCC);
                     }
                     if (
                        pads[j].origsignal == PAD_D4 ||
                        pads[j].origsignal == PAD_D2 ||
                        pads[j].origsignal == PAD_D1
                        ) {
                        pads[j].SetInputSignal(SIG_GND);
                     }
               }
               else if (pom_rd) // nothing is read
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
         printf("       : C// // // // // AAAA AA                      \n");
         printf("       : LRH IN MR RW MI 1111 11AA AAAA AAAA DDDD DDDD\n");
         printf("       : KSL NM 1F DR QQ 5432 1098 7654 3210 7654 3210\n");
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
            if (pads[j].origsignal == PAD__INT)
            {
               printf("%c", pom2);
            }
            if (pads[j].origsignal == PAD__NMI)
            {
               printf("%c ", pom2);
            }
            if (pads[j].origsignal == PAD__M1)
            {
               pom_m1 = (pom2 == '1');
               printf("%c", pom2);
            }
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

         printf(" IFF:%c", (transistors[sig_iff1].IsOn()) ? '1' : '0');
         printf("%c",      (transistors[sig_iff2].IsOn()) ? '1' : '0');

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

#if 0
      if ((i % DIVISOR) == (DIVISOR - 1)) {
         int pc = GetRegVal(reg_pch) << 8 | GetRegVal(reg_pcl);
         printf("## ========================================================================\n");
         printf("## PC=%04X\n",pc);

#if 0
         for (int sn = 0; sn < signals.size(); sn++) {
            float val = signals[sn].ReadOutput();
            char sv = '?';
            if (val < -0.05f) {
               sv = '0';
            } else if (val > 0.05f) {
               sv = '1';
            }
            printf("## %d = %c\n", sn, sv);
         }
#endif
         for (int tn = 2063; tn < transistors.size(); tn++) {
            printf("## %d = %d (%04x)\n", tn, transistors[tn].IsOn(), pc);
         }
      }
#endif

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
