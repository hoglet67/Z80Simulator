#include <map>

#define MAX_EDGES 80
#define MAX_ITERS 10

extern int Instances;

//////////////////////////////////////////////////////////////////////////////

enum SPECIAL_NODES {
   NODE_vcc = 2,
   NODE_vss = 1,
   NODE_clk = 3
};

enum VERTEX_TYE {
   VT_NET = 0,
   VT_NET_EXT,
   VT_EFET,
   VT_DFET,
   VT_EFET_VCC,
   VT_EFET_VSS,
   VT_EFET_LATCH,
   VT_MODULE_BASE = 100
};

enum TERM_TYPE {
   TT_GATE = 2,
   TT_CHAN,
   TT_OUT,
   TT_IN,
   TT_PAD
};

enum {
   LST_NETS = 0,
   LST_DEVS = 1,
   LST_SIZE = 2
};

enum {
   CMD_VERILOG = 0,
   CMD_PORTS,
   CMD_LOGIC,
   CMD_0ADL_H1x1,
   CMD_UNARY
};

//////////////////////////////////////////////////////////////////////////////

struct INTERFACE {
   virtual void AddPort(int node, int term_type, int hint=0) = 0;
};

struct INTERFACE1 : INTERFACE {
   virtual int   GetMate(int) = 0;
   virtual char* GetRoot(int) = 0;
   virtual int   GetKeep(int) = 0;
};

struct INTERFACE2 : INTERFACE {
   virtual void AddPull(int node) = 0;
   virtual void AddTran(int tran, int gate, int drain, int source) = 0;
};

//////////////////////////////////////////////////////////////////////////////

struct BASE {
   int label, degree;
   union {
      bool valid, safe; // phase 1, phase 2
   };
   
   bool IsSpecial() {return !degree;}
   
   virtual void AppendEdge(BASE *, int term_type) = 0;
   virtual void RemoveEdge(BASE *) = 0;
   
BASE() : degree(0) {Instances++;}
   virtual ~BASE()    {Instances--;}
};

//////////////////////////////////////////////////////////////////////////////

struct VERT : BASE {
   int   type, id;
   int   term_types[MAX_EDGES];
   BASE *neighbours[MAX_EDGES];
   VERT *next, *prev, *mate;
   
   virtual void Init_Phase1() = 0;
   virtual void Init_Phase2(int type=VT_NET);
   
   void AppendEdge(BASE *, int term_type);
   void RemoveEdge(BASE *);
   void InsertVertex(VERT **);
   void RemoveVertex(VERT **);
   bool Relabel_Phase1();
   void Relabel_Phase2();
   
VERT(int t, int i) :  type(t), id(i) {Instances++;}
   virtual ~VERT()                      {Instances--;}
};
//////////////////////////////////////////////////////////////////////////////

struct GRAPH : INTERFACE2 {
   std::map<int,BASE*> net_idx;
   VERT *list[LST_SIZE];
   
   void AddDevice(int type, int id=0);
   void AddTerminal(int node, int term_type);
   void AddTerminal(BASE *net, int term_type);
   void AddPort(int node, int term_type, int hint=0);
   void AddPull(int node);
   void AddTran(int tran, int gate, int drain, int source);
   void AddSpecial(int node);
   void AddSpecials();
   void DeleteSpecial(BASE *net);
   void DeleteSpecials();
   void RemoveVertex(VERT *, int);
   
   void  Init_Phase1();
   void  Init_Phase2();
   
   GRAPH(); ~GRAPH();
};

//////////////////////////////////////////////////////////////////////////////

int Replace(GRAPH& cpu, GRAPH& sub, int ord);

void error(int i);
