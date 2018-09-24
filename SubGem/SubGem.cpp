#pragma warning(disable:4786)

#include <stdlib.h>

#include <iterator>
#include <list>
#include <map>

#include "SubGem.h"

//////////////////////////////////////////////////////////////////////////////

struct SPECIAL_NET : BASE { // Vcc, Vss, cp1, cp2
   int edges;
   SPECIAL_NET(int node) : edges(0) {
      valid = true; // safe = true
      label = node; // Never re-labelled
   }
   ~SPECIAL_NET() {
      if (edges) error(10);
   }
   void AppendEdge(BASE *, int) {edges++;}
   void RemoveEdge(BASE *)      {edges--;}
};

struct INTERNAL_NET : VERT {
   INTERNAL_NET(int node) : VERT(VT_NET, node) {}
   void Init_Phase1();
};

struct EXTERNAL_NET : VERT { // Subcircuit ports
   int term_type, hint;
   EXTERNAL_NET(int node, int port_type, int hint) : VERT(VT_NET_EXT, node), term_type(port_type) {
      this->hint=hint;
   }
   void Init_Phase1();
};

struct DEVICE : VERT {
   DEVICE(int type, int id) : VERT(type, id) {}
   void Init_Phase1();
   void Init_Phase2();
};

//////////////////////////////////////////////////////////////////////////////
// Saving and restore vertex states around recursive call in phase 2

#define TYPE int64_t

struct STATE {
   TYPE *data[4];
   void Capture(GRAPH**);
   void Restore(GRAPH**);
   STATE()  {int i; for (i=0; i<4; i++) data[i]=0;}
   ~STATE() {int i; for (i=0; i<4; i++) if (data[i]) delete data[i];}
};

void STATE::Capture(GRAPH *g[]) {
   VERT *v, *l;
   int i, j;
   for (i=0; i<4; i++) {
      l = g[i/2]->list[i%2];
      for (j=0, v=l; v; v=v->next) j++;
      data[i] = new TYPE [j*3];
      for (j=0, v=l; v; v=v->next) {
         data[i][j++] = v->safe;
         data[i][j++] = v->label;
         data[i][j++] = (TYPE) v->mate;
      }
   }
}

void STATE::Restore(GRAPH *g[]) {
   VERT *v;
   int i, j;
   for (i=0; i<4; i++) {
      for (j=0, v=g[i/2]->list[i%2]; v; v=v->next) {
         v->safe = data[i][j++]? true:false;
         v->label = data[i][j++];
         v->mate = (VERT*) data[i][j++];
      }
   }
}

//////////////////////////////////////////////////////////////////////////////
// Vertices

void VERT::AppendEdge(BASE *neighbour, int term_type) {
   if (degree==MAX_EDGES) error(11);
   term_types[degree]   = term_type;
   neighbours[degree++] = neighbour;
}

void VERT::RemoveEdge(BASE *neighbour) {
   for (int i=0; i<degree; i++) if (neighbours[i]==neighbour) {
         if (i < --degree) {
            neighbours[i] = neighbours[degree];
            term_types[i] = term_types[degree];
         }
         return;
      }
   error(12);
}

void VERT::RemoveVertex(VERT **lnk) {
   for (int i=0; i<degree; i++) neighbours[i]->RemoveEdge(this);
   if (next) next->prev = prev;
   if (prev) prev->next = next;
   if (*lnk==this) *lnk = next;
}

void VERT::InsertVertex(VERT **lnk) {
   prev = 0;
   next = *lnk;
   *lnk = this;
   if (next) next->prev = this;
}

//////////////////////////////////////////////////////////////////////////////
// Netlists

GRAPH::GRAPH() {
   for (int i=0; i<LST_SIZE; i++) list[i]=0;
   AddSpecials();
}

GRAPH::~GRAPH() {
   for (int i=0; i<LST_SIZE; i++) while(list[i]) RemoveVertex(list[i], i);
   DeleteSpecials();
}

void GRAPH::RemoveVertex(VERT *v, int type) {
   v->RemoveVertex(list+type);
   delete v;
}

void GRAPH::AddDevice(int type, int id) {
   VERT *dev = new DEVICE(type, id);
   dev->InsertVertex(list+LST_DEVS);
}

void GRAPH::AddTerminal(BASE *net, int term_type) {
   BASE *dev = list[LST_DEVS]; // Last one added
   dev->AppendEdge(net, term_type);
   net->AppendEdge(dev, term_type);
}

void GRAPH::AddTerminal(int node, int term_type) {
   std::map<int,BASE*>::iterator it = net_idx.find(node);
   if (it!=net_idx.end()) {
      BASE *net = it->second;
      AddTerminal(net, term_type);
   }
   else {
      VERT *net = new INTERNAL_NET(node);
      net_idx[node] = net;
      net->InsertVertex(list+LST_NETS);
      AddTerminal(net, term_type);
   }
}

void GRAPH::AddPort(int node, int port_type, int hint) {
   VERT *net = new EXTERNAL_NET(node, port_type, hint);
   net_idx[node] = net;
   net->InsertVertex(list+LST_NETS);
}

void GRAPH::AddSpecial(int node) {
   BASE *net = new SPECIAL_NET(node);
   net_idx[node] = net;
}

void GRAPH::AddSpecials() {
   AddSpecial(NODE_vcc);
   AddSpecial(NODE_vss);
   AddSpecial(NODE_cp1);
   AddSpecial(NODE_cp2);
}

void GRAPH::DeleteSpecial(BASE *net) {
   if (net->IsSpecial()) delete net;
   else error(13);
}

void GRAPH::DeleteSpecials() {
   DeleteSpecial(net_idx[NODE_vcc]);
   DeleteSpecial(net_idx[NODE_vss]);
   DeleteSpecial(net_idx[NODE_cp1]);
   DeleteSpecial(net_idx[NODE_cp2]);
}

void GRAPH::AddPull(int node) {
   AddDevice(VT_DFET, node+1000000);
   AddTerminal(node,     TT_GATE);
   AddTerminal(node,     TT_CHAN);
   AddTerminal(NODE_vcc, TT_CHAN);
}

void GRAPH::AddTran(int tran, int gate, int chan0, int chan1) {
   if      (chan0==NODE_vcc || chan1==NODE_vcc) AddDevice(VT_EFET_VCC, tran);
   else if (chan0==NODE_vss || chan1==NODE_vss) AddDevice(VT_EFET_VSS, tran);
   else                                         AddDevice(VT_EFET,     tran);
   AddTerminal(gate,  TT_GATE);
   AddTerminal(chan0, TT_CHAN);
   AddTerminal(chan1, TT_CHAN);
}

//////////////////////////////////////////////////////////////////////////////
// Phase 1 vertex labelling

void INTERNAL_NET::Init_Phase1() {
   valid = true;
   label = degree;
}

void EXTERNAL_NET::Init_Phase1() {
   valid = false;
   label = 0; // Not used
}

void DEVICE::Init_Phase1() {
   valid = true;
   label = type;
}

void GRAPH::Init_Phase1() {
   for (int i=0; i<LST_SIZE; i++)
      for (VERT *v=list[i]; v; v=v->next)
         v->Init_Phase1();
}

bool VERT::Relabel_Phase1() {
   if (valid) {
      for (int j=0; j<degree; j++) {
         label += neighbours[j]->label * term_types[j];
         valid &= neighbours[j]->valid;
      }
   }
   return valid;
}

//////////////////////////////////////////////////////////////////////////////
// Sub Gemini Phase 1

void Phase1(
   GRAPH            &cpu,
   GRAPH            &sub,
   std::list<VERT*> &candidates,
   VERT *           &key,
   int              &type
   ) {
   // Initially, all vertices in both graphs are partitioned into equivalence classes according to labels based on vertex invariants:
   // All device vertices are labeled according to their type, and all net vertices are labeled according to their degree.

   cpu.Init_Phase1();
   sub.Init_Phase1();

   // SubGemini iteratively relabels vertices until either all net vertices or all device vertices in S become corrupt

   type = LST_NETS;
   VERT *v;

   for (;;) {
      for (v=cpu.list[type]; v; v=v->next) v->Relabel_Phase1();
      bool valid = false;
      for (v=sub.list[type]; v; v=v->next) valid |= v->Relabel_Phase1();
      type = LST_NETS+LST_DEVS - type;
      if (!valid) break;
   }

   // We are left with a number of valid partitions in S and the corresponding partitions in g.

   std::map<int,int> parts;
   std::map<int,int>::iterator it;

   for (v=sub.list[type]; v; v=v->next) if (v->valid) {
         it = parts.find(v->label);
         if (it==parts.end()) parts[v->label]=0;
      }

   for (v=cpu.list[type]; v; v=v->next) {
      it = parts.find(v->label);
      if (it!=parts.end()) it->second++;
   }

   // The smallest of these partitions is chosen as the candidate vector so that the least amount of work has to be done in Phase II.

   int label=0, size=1<<30;
   for (it=parts.begin(); it!=parts.end(); it++) if (it->second && it->second<size) {
         label = it->first;
         size = it->second;
      }

   for (v=cpu.list[type]; v; v=v->next) if (v->label==label)
                                           candidates.push_back(v);

   // The valid vertex from S with the same label as that of the candidate partition is chosen as the key vertex.
   // If multiple valid vertices from S have the same label at this point, an arbitrary one can be chosen to be K.

   for (v=sub.list[type]; v; v=v->next) if (v->label==label && v->valid) key = v;
}

//////////////////////////////////////////////////////////////////////////////
// Phase 2 vertex labelling

void VERT::Init_Phase2(int type) { // type=VT_NET (default) for nets
   label = type; // can't use net degree - might be EXTERNAL_NET
   safe = false;
   mate = NULL;
}

void DEVICE::Init_Phase2() {
   VERT::Init_Phase2(type);
}

void GRAPH::Init_Phase2() {
   for (int i=0; i<LST_SIZE; i++)
      for (VERT *v=list[i]; v; v=v->next)
         v->Init_Phase2();
}

void VERT::Relabel_Phase2() {
   if (mate==NULL)
      for (int j=0; j<degree; j++)
         if (neighbours[j]->safe)
            label += neighbours[j]->label * term_types[j];
}

enum {CPU, SUB, NUM_GRAPHS};

struct PARTITION {
   int tally[NUM_GRAPHS];
};

//////////////////////////////////////////////////////////////////////////////
// Sub Gemini Phase 2

bool Phase2(
   GRAPH           & cpu,
   GRAPH           & sub,
   VERT *            key,
   int               type
   ) {
   GRAPH *g[NUM_GRAPHS] = {&cpu, &sub};
   VERT *v;
   int i;
/*
  SubGemini does the following procedure for each instance c from the
  candidate vector in turn. It first assumes that c is the image of K.
  It marks these two vertices matched and gives them both the same random,
  unique label which does not change. Starting from this one label, it then
  iteratively relabels the surrounding vertices in both graphs.
*/
   key->mate->label = key->label = 1+rand();
   key->mate->safe  = key->safe  = true;
   key->mate->mate  = key;

   for(int iter=MAX_ITERS; iter>0; iter--) {

      type = LST_NETS+LST_DEVS - type;

      std::map<int,PARTITION> parts;
      std::map<int,PARTITION>::iterator it;

      // Re-label
      for (i=0; i<NUM_GRAPHS; i++)
         for (v=g[i]->list[type]; v; v=v->next)
            v->Relabel_Phase2();

      // Re-partition
      for (i=0; i<NUM_GRAPHS; i++)
         for (v=g[i]->list[type]; v; v=v->next) {
            it = parts.find(v->label);
            if (it==parts.end()) {
               parts[v->label].tally[  i]=1;
               parts[v->label].tally[1-i]=0;
            }
            else
               it->second.tally[i]++;
         }

      // Check progress
      for (it=parts.begin(); it!=parts.end(); it++) {
         if (it->second.tally[CPU]==it->second.tally[SUB]) { // Equal partition sizes in G and S?
            VERT *vv[NUM_GRAPHS];
            for (i=0; i<NUM_GRAPHS; i++)
               for (v=g[i]->list[type]; v; v=v->next)
                  if (v->label==it->first) {
                     vv[i]=v;
                     if (v->safe==false) {
                        v->safe = true; // Safe to use label
                        iter=MAX_ITERS;
                     }
                  }

            if (it->second.tally[CPU]==1) { // Singletons?
               if (vv[SUB]->mate==NULL
                   && (vv[SUB]->type==VT_NET_EXT || vv[CPU]->degree==vv[SUB]->degree)
                  ) {
                  vv[CPU]->mate = vv[SUB]; // Matched
                  vv[SUB]->mate = vv[CPU];
                  vv[CPU]->label = vv[SUB]->label = 1+rand(); // New (unique?) random label
                  iter=MAX_ITERS;
               }
            }
         }
/*
  If no progress has been made after some iteration, choose an unmatched vertex s from S and
  the partition P in G with the same label as s, and call VerifyImage(s,P) recursively ....
  ..... upon each recursive call, the current state including labels and matches is saved.
*/
         if (iter<3 // No progress?
             && it->second.tally[SUB]>1 && it->second.tally[CPU]<20 && it->second.tally[CPU]>=it->second.tally[SUB]) {
            // Choose unmatched vertex
            for (v=sub.list[type]; v; v=v->next) if (v->label==it->first) break;
            key = v;
            // VerifyImage(s,P)
            for (v=cpu.list[type]; v; v=v->next) if (v->label==it->first) {
                  STATE st;
                  st.Capture(g);
                  key->mate = v;
                  if (Phase2(cpu, sub, key, type)) return true;
                  st.Restore(g);
               }
            return false;
         }
      }

      int unmatched=0;
      for (i=0; i<LST_SIZE; i++) for (v=sub.list[i]; v; v=v->next) if (v->mate==NULL) unmatched++;
      if (unmatched==0) return true;
   }
   return false;
}

void MatchHints( // cheating
   GRAPH& cpu,
   GRAPH& sub) {

   for (VERT *v=sub.list[LST_NETS]; v; v=v->next) {
      if (v->type!=VT_NET_EXT) continue;
      EXTERNAL_NET *ext = (EXTERNAL_NET*) v;
      if (ext->hint==0) continue;
      v->mate = (VERT*) cpu.net_idx[ext->hint];
      v->mate->label = v->label = 1+rand();
      v->mate->safe  = v->safe  = true;
      v->mate->mate  = v;
   }
}

bool Phase2(
   GRAPH           & cpu,
   GRAPH           & sub,
   std::list<VERT*>& candidates,
   VERT *            key,
   int               type
   ) {
   while (candidates.size()) {
      cpu.Init_Phase2();
      sub.Init_Phase2();
      key->mate = candidates.front();
      candidates.pop_front();
      MatchHints(cpu, sub);
      if (Phase2(cpu, sub, key, type)) return true;
   }
   return false;
}

//////////////////////////////////////////////////////////////////////////////
// Replace all instances of sub in cpu with module of type ord

int Replace(GRAPH& cpu, GRAPH& sub, int ord) {
   srand(1);

   std::list<VERT*> candidates;
   VERT *v, *key;
   int type, n=0;

   for(Phase1(cpu, sub, candidates, key, type);
       Phase2(cpu, sub, candidates, key, type);
       n++
      ) {
      for (v=sub.list[LST_DEVS]; v; v=v->next) {
         candidates.remove(v->mate);
         cpu.RemoveVertex(v->mate, LST_DEVS); // Remove devices
      }

      cpu.AddDevice(ord); // Replacement

      for (v=sub.list[LST_NETS]; v; v=v->next) {
         if (v->type==VT_NET_EXT) {
            EXTERNAL_NET *ext = (EXTERNAL_NET*) v;
            cpu.AddTerminal(v->mate, ext->term_type); // Connect
         }
         else {
            candidates.remove(v->mate);
            cpu.RemoveVertex(v->mate, LST_NETS); // Remove internal net
         }
      }
   }
   return n;
}
