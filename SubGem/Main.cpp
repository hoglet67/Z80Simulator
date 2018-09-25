#pragma warning(disable:4786)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <set>
#include <inttypes.h>

#define __int64 int64_t

#include "SubGem.h"

#define NUM_TRANS 6810
#define NUM_NODES 3599

char *Module(INTERFACE *sub, int ord, int cmd, FILE *fp);

//////////////////////////////////////////////////////////////////////////////
// These nodes must not be removed by inverter/buffer optimisation

int Required[] = {
   0
};

//////////////////////////////////////////////////////////////////////////////
// Self-checking

int Instances;

void error(int i) {
   fprintf(stderr, "ERROR %d\n", i);
   exit(-1);
}

//////////////////////////////////////////////////////////////////////////////
// Detect duplicate transistors

struct TRAN_SET {
   bool UniqueTran(int g, int c0, int c1) {
      __int64 i64 = g;
      i64<<=20; i64 += c0<c1? c0:c1;
      i64<<=20; i64 += c0<c1? c1:c0;
      std::pair<std::set<__int64>::iterator,bool> ret = s.insert(i64);
      return ret.second;
   }
private:
   std::set<__int64> s;
};

//////////////////////////////////////////////////////////////////////////////
// Load 6502 netlist from js files

void LoadNetlist(GRAPH& cpu) {
   char s[200];
   char ch = 0;
   int i, tran, gate, node, chan[2];
   TRAN_SET ts;

   FILE *fp = fopen("transdefs.js", "r");
   if (!fp) error(1);
   for (i=0; i<NUM_TRANS; i++) {
      for (;;) {
         if (!fgets(s, sizeof s, fp)) error(2);
         if (sscanf(s, "['t%d',%d,%d,%d,", &tran, &gate, chan+0, chan+1)==4) break;
      }
//      if (gate==NODE_vss && (chan[0]==NODE_cp2 || chan[1]==NODE_cp2)) continue; // Termination?
      if (chan[0]==chan[1]) continue; // Miller slow-down capacitors on db2, db4, db6
      if (ts.UniqueTran(gate, chan[0], chan[1])==false) continue; // Parallel
      cpu.AddTran(tran, gate, chan[0], chan[1]);
   }
   fclose(fp);

   char *pulls = new char[NUM_NODES];
   memset(pulls, 0, NUM_NODES);

   fp = fopen("segdefs.js", "r");
   if (!fp) error(3);
   for (;;) {
      if (!fgets(s, sizeof s, fp)) error(4);
      if (!strcmp(s, "var segdefs = [\n")) break;
   }
   for (;;) {
      if (!fgets(s, 10, fp)) return;
      if (s[0]==']') break;
      if (sscanf(s, "[%d,'%c',", &node, &ch)!=2) error(5);
      if (node<0 || node>=NUM_NODES) error(6);
      if (ch=='+') pulls[node]=1; else if (ch!='-') error(7);
      while (!feof(fp) && fgetc(fp)!='\n');
   }
   fclose(fp);

   for (node=0; node<NUM_NODES; node++) if (pulls[node]) cpu.AddPull(node);
   delete pulls;

   VERT *v, *n;

   // Delete unused transistors: t1322, t2730, t2863
   for (v=cpu.list[LST_NETS]; v; v=n) {
      n = v->next;
      if (v->degree<2) cpu.RemoveVertex(v, LST_NETS);
   }
   for (v=cpu.list[LST_DEVS]; v; v=n) {
      n = v->next;
      if (v->degree<3) cpu.RemoveVertex(v, LST_DEVS);
   }
}

//////////////////////////////////////////////////////////////////////////////
// Optimise inverters/buffers

struct _Net {
   int src, inv, opt, rqd;
}
   Net[NUM_NODES];

void UnaryBuf(int driven, int driver) {
   Net[driven].src = driver;
   Net[driven].inv = 0;
   Net[driven].opt = 1;
}

void UnaryInv(int driven, int driver) {
   Net[driven].src = driver;
   Net[driven].inv = 1;
   Net[driven].opt = 1;
}

int GetRoot(char *p, int n) {
   return 1+sprintf(p, Net[n].inv? "~i[%d]":"i[%d]",
                    Net[n].opt? Net[n].src : n);
}

int GetKeep(int n) {
   return Net[n].rqd || Net[n].opt==0;
}

//////////////////////////////////////////////////////////////////////////////
// Convert subcircuit to main circuit node numbers

struct MAPPING : INTERFACE1 {
   std::map<int,int> idx;
   char s[1024], *p;
   VERT *dev;
   int ports;

   void AddPort(int node, int, int) {
      idx[node] = ((VERT*)dev->neighbours[--ports])->id;
   }
   int GetMate(int node) {
      return idx[node];
   }
   char *GetRoot(int node) {
      char *pos=p;
      p += ::GetRoot(p, idx[node]);
      return pos;
   }
   int GetKeep(int node) {
      return ::GetKeep(idx[node]);
   }
   MAPPING(VERT *v) : p(s), dev(v), ports(v->degree) {
      Module(this, v->type, CMD_PORTS, 0);
   }
};

//////////////////////////////////////////////////////////////////////////////

void Get_0ADL_H1x1(GRAPH& cpu) { // Tidy-up loose ends
   VERT *v;
   for (v=cpu.list[LST_DEVS]; v; v=v->next) {
      if (v->type<VT_MODULE_BASE) continue;
      MAPPING m(v);
      Module(&m, v->type, CMD_0ADL_H1x1, 0);
   }
}

void OptimiseBuffers(GRAPH& cpu) { // Inverter/buffer optimisation
   int i, n, d;
   VERT *v;

   // Gather connectivity of inverters and buffers
   for (v=cpu.list[LST_DEVS]; v; v=v->next) {
      if (v->type<VT_MODULE_BASE) continue;
      MAPPING m(v);
      Module(&m, v->type, CMD_UNARY, 0);
   }
   // Nets which should not be optimised away
   for (i=0; Required[i]; i++) {
      n = Required[i];
      Net[n].rqd = 1;
      Net[n].opt = 0;
   }
   // Trace driver of buffer/inverter chains
   for (n=0; n<NUM_NODES; n++) {
      for(i=0, d=n; Net[d].opt; d=Net[d].src) i^=Net[d].inv;
      Net[n].src = d;
      Net[n].inv = i;
   }
}

void FindModules(GRAPH& cpu) { // SubGemini top-level
   int ord = VT_MODULE_BASE;
   for (;;) {
      GRAPH sub;
      char *s = Module(&sub, ord, CMD_PORTS, 0); // Define ports
      if (s==0) break;
      Module(&sub, ord, CMD_LOGIC, 0); // Transistors and wires
      int n = Replace(cpu, sub, ord++); // Call SubGemini
      printf("%4d  %s\n", n, s);
   }
}

void DumpModules(GRAPH& cpu, FILE *fp) {
   VERT *v;
   for (v=cpu.list[LST_DEVS]; v; v=v->next) {
      if (v->type<VT_MODULE_BASE) continue;
      MAPPING m(v);
      Module(&m, v->type, CMD_VERILOG, fp); // Write Verilog
   }
}

//////////////////////////////////////////////////////////////////////////////
// Transparent latches clocked by cp1, cp2

int Floatable(VERT *v) {
   for (int i=0; i<v->degree; i++) if (v->term_types[i]==TT_OUT) return 0;
   return 1;
}

void DumpLatches(GRAPH& cpu, FILE *fp) {
   VERT *t, *c[2];
   BASE *g;
   char s[2][80];
   int r;

   for (t=cpu.list[LST_DEVS]; t; t=t->next) if (t->type==VT_EFET) {
         g = t->neighbours[0];
         if (!g->IsSpecial()) error(8);
         c[0] = (VERT*) t->neighbours[1];
         c[1] = (VERT*) t->neighbours[2];
         r = Floatable(c[1]);

         sprintf(s[0], "i[%d]", g->label); // cp1 or cp2
         GetRoot(s[1], c[1-r]->id);
         fprintf(fp, "assign o[%d] = %s ? %s : i[%d];\n", c[r]->id, s[0], s[1], c[r]->id);

         //t->type = VT_EFET_LATCH;
      }
}

//////////////////////////////////////////////////////////////////////////////
// Check all done

int NumTrans(GRAPH& cpu, int type) {
   int n=0;
   VERT *v;
   for (v=cpu.list[LST_DEVS]; v; v=v->next) if (type==v->type) n++;
   return n;
}

void Stats(GRAPH& cpu) {
   fprintf(stderr, "%4d  VT_EFET_VCC\n",  NumTrans(cpu, VT_EFET_VCC));
   fprintf(stderr, "%4d  VT_EFET_VSS\n",  NumTrans(cpu, VT_EFET_VSS));
   fprintf(stderr, "%4d  VT_EFET\n",      NumTrans(cpu, VT_EFET));
   fprintf(stderr, "%4d  VT_DFET\n",      NumTrans(cpu, VT_DFET));
}

//////////////////////////////////////////////////////////////////////////////

int main() {
   GRAPH cpu;
   FILE *fp;

   LoadNetlist(cpu);
   FindModules(cpu); // SubGemini algorithm
   Get_0ADL_H1x1(cpu);

   fp = fopen("logic_unopt.inc", "w"); // For simulation
   DumpModules(cpu, fp);
   //DumpLatches(cpu, fp);
   fclose(fp);

   OptimiseBuffers(cpu);// Remove buffers and inverters

   fp = fopen("logic.inc", "w"); // For synthesis
   DumpModules(cpu, fp);
   //DumpLatches(cpu, fp);
   fclose(fp);

   //Stats(cpu);
   return 0;
}
