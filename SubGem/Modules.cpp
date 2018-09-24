#pragma warning(disable:4786)

#include <stdio.h>
#include <iterator>
#include <map>

#include "SubGem.h"

void UnaryInv(int, int);
void UnaryBuf(int, int);

int GetRoot(char *, int);

const int NODE_H1x1 = 1042;

std::map<int,int> Map_0ADL, Map_H1x1;

//////////////////////////////////////////////////////////////////////////////
// Simple gates encoded by height of transistor stack

const char *Gates[] = {
    "~2222",
    "~2221",
    "~222",
    "~2211111",
    "~22",
    "~2111111",
    "~211111",
    "~21111",
    "~2111",
    "~211",
    "~21",
    "~2",
    "~111111111",
    "~11111111",
    "~1111111",
    "~111111",
    "~11111",
    "~1111",
    "~111",
    "~11",
    "~1"
};

const int NumGates = sizeof(Gates)/sizeof(char*);

//////////////////////////////////////////////////////////////////////////////

void GatePorts(INTERFACE1 *sub, const char *p) {
    for (int x=0; p[x]; x++) {
        if (p[x]=='1') {
            sub->AddPort(100+x, TT_IN);
        }
        else { // '2'
            sub->AddPort(100+x, TT_IN);
            sub->AddPort(110+x, TT_IN);
        }
    }
}

void GateLogic(INTERFACE2 *sub, const char *p) {
    sub->AddPull(300);
    for (int x=0; p[x]; x++) {
        if (p[x]=='1') {
            sub->AddTran(400+x, 100+x, 300, NODE_vss);
        }
        else { // '2'
            sub->AddTran(400+x, 100+x, 200+x, 300);
            sub->AddTran(410+x, 110+x, 200+x, NODE_vss);
        }
    }
}

void GateVerilog(INTERFACE1 *sub, const char *p, FILE *fp) {
    for (int x=0; p[x]; x++) {
        if (x) fputc('|', fp);
        if (p[x]=='1')
            fprintf(fp, "%s", sub->GetRoot(100+x));
        else // '2'
            fprintf(fp, "%s&%s", sub->GetRoot(100+x), sub->GetRoot(110+x));
    }
}

void GateLogic(GRAPH *sub, const char *p, int ord, int inv) { // 2nd pass
    // Inverter
    sub->AddDevice(inv, 500);
    sub->AddTerminal(300, TT_OUT);
    sub->AddTerminal(301, TT_IN);
    // Gate fouund by 1st pass
    sub->AddDevice(ord, 501);
    sub->AddTerminal(301, TT_OUT);
    for (int x=0; p[x]; x++) {
        if (p[x]=='1') {
            sub->AddTerminal(100+x, TT_IN);
        }
        else { // '2'
            sub->AddTerminal(100+x, TT_IN);
            sub->AddTerminal(110+x, TT_IN);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
// Define sub-circuits

const char *Module(INTERFACE *sub, int ord, int cmd, FILE *fp) {
    int idx=VT_MODULE_BASE;

    INTERFACE1 *sub1 = (INTERFACE1*) sub;
    INTERFACE2 *sub2 = (INTERFACE2*) sub;

    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp,
                    "MUX #(2) mux_alua_%d ("
                    ".o(o[%d]), .i(%s), "
                    ".s({%s,%s}), "
                    ".d({%s,i[%d]}));\n",
                    sub1->GetMate(103), sub1->GetMate(103), sub1->GetRoot(103),
                    sub1->GetRoot(110), sub1->GetRoot(111),
                    sub1->GetRoot(106), NODE_vss);
                fprintf(fp,
                    "MUX #(3) mux_alub_%d ("
                    ".o(o[%d]), .i(%s), "
                    ".s({ %s,%s,%s}), "
                    ".d({~%s,%s,%s}));\n",
                    sub1->GetMate(102), sub1->GetMate(102), sub1->GetRoot(102),
                    sub1->GetRoot(107), sub1->GetRoot(108), sub1->GetRoot(109),
                    sub1->GetRoot(104), sub1->GetRoot(104), sub1->GetRoot(105));
                fprintf(fp, "assign o[%d] = ~(%s|%s);\n",
                    sub1->GetMate(100), sub1->GetRoot(102), sub1->GetRoot(103));
                fprintf(fp, "assign o[%d] = ~(%s&%s);\n",
                    sub1->GetMate(101), sub1->GetRoot(102), sub1->GetRoot(103));
                break;
            case CMD_PORTS:
                sub1->AddPort(100, TT_OUT); // NOR
                sub1->AddPort(101, TT_OUT); // NAND
                sub1->AddPort(102, TT_OUT); // alub storage node
                sub1->AddPort(103, TT_OUT); // alua storage node
                sub1->AddPort(104, TT_IN);  // idb
                sub1->AddPort(105, TT_IN);  // adl
                sub1->AddPort(106, TT_IN);  // sb
                sub1->AddPort(107, TT_IN);  // alub = ~idb
                sub1->AddPort(108, TT_IN);  // alub = idb
                sub1->AddPort(109, TT_IN);  // alub = adl
                sub1->AddPort(110, TT_IN);  // alua = sb
                sub1->AddPort(111, TT_IN);  // alua = 0
                break;
            case CMD_LOGIC:
                sub2->AddPull(100);
                sub2->AddPull(101);
                sub2->AddPull(112);
                sub2->AddTran(200, 102, 100, NODE_vss);
                sub2->AddTran(201, 103, 100, NODE_vss);
                sub2->AddTran(202, 102, 113, 101);
                sub2->AddTran(203, 103, 113, NODE_vss);
                sub2->AddTran(204, 107, 102, 112);
                sub2->AddTran(205, 108, 102, 104);
                sub2->AddTran(206, 109, 102, 105);
                sub2->AddTran(207, 110, 103, 106);
                sub2->AddTran(208, 111, 103, NODE_vss);
                sub2->AddTran(209, 104, 112, NODE_vss);
        }
        return "ALU inputs";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp,
                    "MUX #(5) mux_alu_out_%d (.o(o[%d]), .i(%s), "
                    ".s({%s&i[%d],%s&i[%d],%s&i[%d],%s&i[%d],%s&i[%d]}), "
                    ".d({%s,%s,%s,%s,i[%d]}));\n",
                    sub1->GetMate(109), sub1->GetMate(109), sub1->GetRoot(109),
                    sub1->GetRoot(100), NODE_cp2,
                    sub1->GetRoot(101), NODE_cp2,
                    sub1->GetRoot(102), NODE_cp2,
                    sub1->GetRoot(103), NODE_cp2,
                    sub1->GetRoot(104), NODE_cp2,
                    sub1->GetRoot(105), sub1->GetRoot(106), sub1->GetRoot(107), sub1->GetRoot(108), NODE_vcc);
                break;
            case CMD_PORTS:
                sub1->AddPort(100, TT_IN);
                sub1->AddPort(101, TT_IN);
                sub1->AddPort(102, TT_IN);
                sub1->AddPort(103, TT_IN);
                sub1->AddPort(104, TT_IN);
                sub1->AddPort(105, TT_IN);
                sub1->AddPort(106, TT_IN);
                sub1->AddPort(107, TT_IN);
                sub1->AddPort(108, TT_IN);
                sub1->AddPort(109, TT_OUT);
                break;
            case CMD_LOGIC:
                sub2->AddTran(200, 100, 110, 105);
                sub2->AddTran(201, 101, 110, 106);
                sub2->AddTran(202, 102, 110, 107);
                sub2->AddTran(203, 103, 110, 108);
                sub2->AddTran(204, 104, 110, NODE_vcc); // SRS
                sub2->AddTran(205, NODE_cp2, 110, 109);
        }
        return "ALU output [7]";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
            fprintf(fp,
                "MUX #(5) mux_alu_out_%d (.o(o[%d]), .i(%s), "
                ".s({%s&i[%d],%s&i[%d],%s&i[%d],%s&i[%d],%s&i[%d]}), "
                ".d({%s,%s,%s,%s,%s}));\n",
                sub1->GetMate(110), sub1->GetMate(110), sub1->GetRoot(110),
                sub1->GetRoot(100), NODE_cp2,
                sub1->GetRoot(101), NODE_cp2,
                sub1->GetRoot(102), NODE_cp2,
                sub1->GetRoot(103), NODE_cp2,
                sub1->GetRoot(104), NODE_cp2,
                sub1->GetRoot(105), sub1->GetRoot(106), sub1->GetRoot(107), sub1->GetRoot(108), sub1->GetRoot(109));
                break;
            case CMD_PORTS:
                sub1->AddPort(100, TT_IN);
                sub1->AddPort(101, TT_IN);
                sub1->AddPort(102, TT_IN);
                sub1->AddPort(103, TT_IN);
                sub1->AddPort(104, TT_IN);
                sub1->AddPort(105, TT_IN);
                sub1->AddPort(106, TT_IN);
                sub1->AddPort(107, TT_IN);
                sub1->AddPort(108, TT_IN);
                sub1->AddPort(109, TT_IN);
                sub1->AddPort(110, TT_OUT);
                break;
            case CMD_LOGIC:
                sub2->AddTran(200, 100, 111, 105);
                sub2->AddTran(201, 101, 111, 106);
                sub2->AddTran(202, 102, 111, 107);
                sub2->AddTran(203, 103, 111, 108);
                sub2->AddTran(204, 104, 111, 109); // SRS
                sub2->AddTran(205, NODE_cp2, 111, 110);
                break;
        }
        return "ALU output [6:0]";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = i[%d]&%s? %s : %s;\n",
                    sub1->GetMate(102), NODE_cp1, sub1->GetRoot(104), sub1->GetRoot(105), sub1->GetRoot(102));
                if (sub1->GetKeep(101)) fprintf(fp, "assign o[%d] = ~%s;\n",
                    sub1->GetMate(101), sub1->GetRoot(102));
                if (sub1->GetKeep(103)) fprintf(fp, "assign o[%d] = %s;\n",
                    sub1->GetMate(103), sub1->GetRoot(102));
                break;
            case CMD_UNARY:
                UnaryInv(sub1->GetMate(101), sub1->GetMate(102));
                UnaryBuf(sub1->GetMate(103), sub1->GetMate(102));
                break;
            case CMD_PORTS:
                sub1->AddPort(101, TT_OUT); // out_n
                sub1->AddPort(102, TT_OUT); // storage node
                sub1->AddPort(103, TT_OUT); // out
                sub1->AddPort(104, TT_IN);  // load (fetch)
                sub1->AddPort(105, TT_IN);  // in
                break;
            case CMD_LOGIC:
                sub2->AddPull(101);
                sub2->AddPull(103);
                sub2->AddTran(201, 102, 101, NODE_vss);
                sub2->AddTran(202, 101, 103, NODE_vss);
                sub2->AddTran(203, NODE_cp2, 102, 103);
                sub2->AddTran(204, NODE_cp1, 105, 106);
                sub2->AddTran(204, 104, 102, 106);
        }
        return "IR";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = %s? %s : %s;\n",
                    sub1->GetMate(102), sub1->GetRoot(104), sub1->GetRoot(106), sub1->GetRoot(102));
                if (sub1->GetKeep(103)) fprintf(fp, "assign o[%d] = %s;\n",
                    sub1->GetMate(103), sub1->GetRoot(102));
                break;
            case CMD_UNARY:
                UnaryBuf(sub1->GetMate(103), sub1->GetMate(102));
                break;
            case CMD_PORTS:
                sub1->AddPort(102, TT_OUT); // storage node
                sub1->AddPort(103, TT_OUT); // out
                sub1->AddPort(104, TT_IN);  // load
                sub1->AddPort(106, TT_IN);  // in
                break;
            case CMD_LOGIC:
                sub2->AddPull(101);
                sub2->AddPull(103);
                sub2->AddTran(201, 102, 101, NODE_vss);
                sub2->AddTran(202, 101, 103, NODE_vss);
                sub2->AddTran(203, NODE_cp2, 102, 103);
                sub2->AddTran(204, 104, 102, 106);
        }
        return "A, X and Y registers";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = %s? %s : %s? %s : %s;\n",
                    sub1->GetMate(300), sub1->GetRoot(303), sub1->GetRoot(304), sub1->GetRoot(305), sub1->GetRoot(301), sub1->GetRoot(300));
                fprintf(fp, "assign o[%d] = i[%d]? ~%s : %s;\n",
                    sub1->GetMate(302), NODE_cp2, sub1->GetRoot(300), sub1->GetRoot(302));
                if (sub1->GetKeep(301)) fprintf(fp, "assign o[%d] = ~%s;\n",
                    sub1->GetMate(301), sub1->GetRoot(302));
                break;
            case CMD_UNARY:
                UnaryInv(sub1->GetMate(301), sub1->GetMate(302));
                break;
            case CMD_PORTS:
                sub1->AddPort(300, TT_OUT); // storage
                sub1->AddPort(301, TT_OUT); // out
                sub1->AddPort(302, TT_OUT); // storage_n
                sub1->AddPort(303, TT_IN);  // load
                sub1->AddPort(304, TT_IN);  // in
                sub1->AddPort(305, TT_IN);  // refresh
                break;
            case CMD_LOGIC:
                sub2->AddPull(301);
                sub2->AddPull(306);
                sub2->AddTran(201, 302, 301, NODE_vss);
                sub2->AddTran(202, 300, 306, NODE_vss);
                sub2->AddTran(203, NODE_cp2, 302, 306);
                sub2->AddTran(204, 303, 300, 304);
                sub2->AddTran(205, 305, 300, 301);
        }
        return "SP";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                if (sub1->GetKeep(102)) fprintf(fp, "assign o[%d] = ~i[%d]; // ~cp1\n",
                    sub1->GetMate(102), NODE_cp1);
                break;
            case CMD_UNARY:
                UnaryInv(sub1->GetMate(102), NODE_cp1);
                break;
            case CMD_PORTS:
                sub1->AddPort(102, TT_OUT);
                    break;
            case CMD_LOGIC:
                sub2->AddPull(101);
                sub2->AddTran(203, NODE_cp1, 102, NODE_vss);
                sub2->AddTran(204, 101, 102, NODE_vcc);
                sub2->AddTran(205, NODE_cp1, 101, NODE_vss);
        }
        return "cp1 inverting buffers";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = i[%d]&%s? %s : %s;\n",
                    sub1->GetMate(106), NODE_cp1, sub1->GetRoot(107), sub1->GetRoot(108), sub1->GetRoot(106));
                break;
            case CMD_PORTS:
                sub1->AddPort(106, TT_PAD);
                sub1->AddPort(107, TT_IN); // load
                sub1->AddPort(108, TT_IN); // data
                break;
            case CMD_LOGIC:
                sub2->AddPull(103);
                sub2->AddPull(104);
                sub2->AddPull(110);
                sub2->AddTran(201, 101, 106, NODE_vcc);
                sub2->AddTran(202, 102, 106, NODE_vss);
                sub2->AddTran(203, 103, 101, NODE_vcc);
                sub2->AddTran(204, 104, 101, NODE_vss);
                sub2->AddTran(205, 104, 102, NODE_vcc);
                sub2->AddTran(206, 103, 102, NODE_vss);
                sub2->AddTran(207, 103, 104, NODE_vss);
                sub2->AddTran(208, 105, 103, NODE_vss);
                sub2->AddTran(209, NODE_cp2, 102, 105);
                sub2->AddTran(210, 107, 105, 109);
                sub2->AddTran(211, NODE_cp1, 109, 110);
                sub2->AddTran(212, 108, 110, NODE_vss);
        }
        return "Address pin";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                if (sub1->GetKeep(101)) fprintf(fp, "assign o[%d] = %s;\n",
                    sub1->GetMate(101), sub1->GetRoot(104));
                break;
            case CMD_UNARY:
                UnaryBuf(sub1->GetMate(101), sub1->GetMate(104));
                break;
            case CMD_PORTS:
                sub1->AddPort(101, TT_PAD);
                sub1->AddPort(104, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(105);
                sub2->AddTran(201, 102, 101, NODE_vcc);
                sub2->AddTran(202, 103, 101, NODE_vss);
                sub2->AddTran(203, 104, 102, NODE_vcc);
                sub2->AddTran(204, 105, 102, NODE_vss);
                sub2->AddTran(205, 105, 103, NODE_vcc);
                sub2->AddTran(206, 104, 103, NODE_vss);
                sub2->AddTran(207, 104, 105, NODE_vss);
        }
        return "RW";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                if (sub1->GetKeep(101)) fprintf(fp, "assign o[%d] = %s;\n",
                    sub1->GetMate(101), sub1->GetRoot(105));
                break;
            case CMD_UNARY:
                UnaryBuf(sub1->GetMate(101), sub1->GetMate(105));
                break;
            case CMD_PORTS:
                sub1->AddPort(101, TT_PAD);
                sub1->AddPort(105, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(103);
                sub2->AddPull(104);
                sub2->AddTran(201, 102, 101, NODE_vcc);
                sub2->AddTran(202, 103, 101, NODE_vss);
                sub2->AddTran(203, 104, 102, NODE_vcc);
                sub2->AddTran(204, 103, 102, NODE_vss);
                sub2->AddTran(205, 103, 104, NODE_vss);
                sub2->AddTran(206, 105, 103, NODE_vss);
        }
        return "SYNC";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = ~(%s|%s&i[%d]);\n",
                    sub1->GetMate(104), sub1->GetRoot(101), sub1->GetRoot(102), NODE_cp2);
                break;
            case CMD_PORTS:
                sub1->AddPort(104, TT_OUT);
                sub1->AddPort(101, TT_IN);
                sub1->AddPort(102, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(104);
                sub2->AddTran(201, 101, 104, NODE_vss);
                sub2->AddTran(202, NODE_cp2, 103, NODE_vss);
                sub2->AddTran(203, 102, 104, 103);
        }
        return "~(1+2.cp2)";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = ~%s;\n", NODE_cp1, sub1->GetRoot(109));
                fprintf(fp, "assign o[%d] =  %s;\n", NODE_cp2, sub1->GetRoot(109));
                break;
            case CMD_PORTS:
                sub1->AddPort(109, TT_PAD); // Needed for phase I to complete
                break;
            case CMD_LOGIC:
                sub2->AddPull(103);
                sub2->AddPull(105);
                sub2->AddPull(106);
                sub2->AddPull(107);
                sub2->AddTran(200, 103, 102, NODE_vcc);
                sub2->AddTran(201, 105, 104, NODE_vcc);
                sub2->AddTran(202, 107, 101, NODE_vcc);
                sub2->AddTran(203, 106, 108, NODE_vcc);
                sub2->AddTran(204, 104, 102, NODE_vss);
                sub2->AddTran(205, 103, 104, NODE_vss);
                sub2->AddTran(206, NODE_cp1, 103, NODE_vss);
                sub2->AddTran(207, 105, 103, NODE_vss);
                sub2->AddTran(208, 109, 105, NODE_vss);
                sub2->AddTran(209, 105, 106, NODE_vss);
                sub2->AddTran(210, 108, 101, NODE_vss);
                sub2->AddTran(211, 107, 108, NODE_vss);
                sub2->AddTran(212, 106, 107, NODE_vss);
                sub2->AddTran(213, NODE_vss, NODE_vss, 109);

                sub2->AddPull(303);
                sub2->AddPull(305);
                sub2->AddPull(306);
                sub2->AddPull(307);
                sub2->AddTran(400, 303, NODE_cp2, NODE_vcc);
                sub2->AddTran(401, 305, 304, NODE_vcc);
                sub2->AddTran(402, 307, NODE_cp1, NODE_vcc);
                sub2->AddTran(403, 306, 308, NODE_vcc);
                sub2->AddTran(404, 304, NODE_cp2, NODE_vss);
                sub2->AddTran(405, 303, 304, NODE_vss);
                sub2->AddTran(406, NODE_cp1, 303, NODE_vss);
                sub2->AddTran(407, 305, 303, NODE_vss);
                sub2->AddTran(408, 109, 305, NODE_vss);
                sub2->AddTran(409, 305, 306, NODE_vss);
                sub2->AddTran(410, 308, NODE_cp1, NODE_vss);
                sub2->AddTran(411, 307, 308, NODE_vss);
                sub2->AddTran(412, 306, 307, NODE_vss);
        }
        return "Clock generator";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = ~%s&i[%d]&~i[%d];\n",
                    sub1->GetMate(101), sub1->GetRoot(102), NODE_cp1, NODE_cp2);
                break;
            case CMD_PORTS:
                sub1->AddPort(101, TT_OUT);
                sub1->AddPort(102, TT_IN);
                sub1->AddPort(103, TT_IN, 43);
                sub1->AddPort(105, TT_IN); // 1247 = ~cp1
                break;
            case CMD_LOGIC:
                sub2->AddPull(104);
                sub2->AddPull(106);
                sub2->AddTran(201, 104, 101, NODE_vcc);
                sub2->AddTran(202, 105, 101, NODE_vss);
                sub2->AddTran(203, 106, 101, NODE_vss);
                sub2->AddTran(204, NODE_cp2, 101, NODE_vss);
                sub2->AddTran(205, 104, 106, NODE_vss);
                sub2->AddTran(206, 102, 104, NODE_vss);
                sub2->AddTran(207, 103, 104, NODE_vss);
        }
        return "datapath gate enables using 43";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = ~%s&i[%d]&~i[%d];\n",
                    sub1->GetMate(101), sub1->GetRoot(102), NODE_cp1, NODE_cp2);
                break;
            case CMD_PORTS:
                sub1->AddPort(101, TT_OUT);
                sub1->AddPort(102, TT_IN);
                sub1->AddPort(105, TT_IN); // 1247 = ~cp1
                break;
            case CMD_LOGIC:
                sub2->AddPull(104);
                sub2->AddPull(106);
                sub2->AddTran(201, 104, 101, NODE_vcc);
                sub2->AddTran(202, 105, 101, NODE_vss);
                sub2->AddTran(203, 106, 101, NODE_vss);
                sub2->AddTran(204, NODE_cp2, 101, NODE_vss);
                sub2->AddTran(205, 104, 106, NODE_vss);
                sub2->AddTran(206, 102, 104, NODE_vss);
                sub2->AddTran(207, NODE_cp2, 104, NODE_vss);
        }
        return "datapath gate enables without 43";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                if (sub1->GetKeep(103)) fprintf(fp, "assign o[%d] = %s;\n",
                    sub1->GetMate(103), sub1->GetRoot(101));
                break;
            case CMD_UNARY:
                UnaryBuf(sub1->GetMate(103), sub1->GetMate(101));
                break;
            case CMD_PORTS:
                sub1->AddPort(103, TT_OUT);
                sub1->AddPort(101, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(102);
                sub2->AddTran(201, 101, 103, NODE_vcc);
                sub2->AddTran(202, 102, 103, NODE_vss);
                sub2->AddTran(203, 101, 102, NODE_vss);
        }
        return "push pull";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                if (sub1->GetKeep(103)) fprintf(fp, "assign o[%d] = ~%s;\n",
                    sub1->GetMate(103), sub1->GetRoot(101));
                break;
            case CMD_UNARY:
                UnaryInv(sub1->GetMate(103), sub1->GetMate(101));
                break;
            case CMD_PORTS:
                sub1->AddPort(103, TT_OUT);
                sub1->AddPort(101, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(102);
                sub2->AddTran(201, 102, 103, NODE_vcc);
                sub2->AddTran(202, 101, 103, NODE_vss);
                sub2->AddTran(203, 101, 102, NODE_vss);
        }
        return "inv push pull";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                if (sub1->GetKeep(101)) fprintf(fp, "assign o[%d] = ~%s;\n",
                    sub1->GetMate(101), sub1->GetRoot(107));
                break;
            case CMD_UNARY:
                UnaryInv(sub1->GetMate(101), sub1->GetMate(107));
                break;
            case CMD_PORTS:
                sub1->AddPort(101, TT_PAD);  // pad
                sub1->AddPort(104, TT_IN);   // tri (RnWstretched)
                sub1->AddPort(107, TT_IN);   // ~in
                break;
            case CMD_LOGIC:
                sub2->AddPull(105);
                sub2->AddPull(106);
                sub2->AddTran(201, 102, 101, NODE_vcc);
                sub2->AddTran(202, 103, 101, NODE_vss);
                sub2->AddTran(203, 104, 102, NODE_vss);
                sub2->AddTran(204, 104, 103, NODE_vss);
                sub2->AddTran(205, 105, 102, NODE_vcc);
                sub2->AddTran(206, 106, 102, NODE_vss);
                sub2->AddTran(207, 106, 103, NODE_vcc);
                sub2->AddTran(208, 105, 103, NODE_vss);
                sub2->AddTran(209, 105, 106, NODE_vss);
                sub2->AddTran(210, 107, 105, NODE_vss);
                sub2->AddTran(213, NODE_vss, NODE_vss, 101);
                sub2->AddTran(215, 104, 106, NODE_vss);
        }
        return "Data I/O";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = ~((%s|%s)&%s);\n",
                    sub1->GetMate(107), sub1->GetRoot(101), sub1->GetRoot(102), sub1->GetRoot(103));
                break;
            case CMD_PORTS:
                sub1->AddPort(107, TT_OUT);
                sub1->AddPort(101, TT_IN);
                sub1->AddPort(102, TT_IN);
                sub1->AddPort(103, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(107);
                sub2->AddTran(201, 101, 105, NODE_vss);
                sub2->AddTran(202, 102, 106, NODE_vss);
                sub2->AddTran(203, 103, 105, 107);
                sub2->AddTran(204, 103, 106, 107);
        }
        return "~(1.3+2.3) (shorted inputs)";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = ~((%s|%s)&(%s|%s)|%s);\n",
                    sub1->GetMate(107), sub1->GetRoot(101), sub1->GetRoot(102), sub1->GetRoot(103), sub1->GetRoot(104), sub1->GetRoot(105));
                break;
            case CMD_PORTS:
                sub1->AddPort(107, TT_OUT);
                sub1->AddPort(101, TT_IN);
                sub1->AddPort(102, TT_IN);
                sub1->AddPort(103, TT_IN);
                sub1->AddPort(104, TT_IN);
                sub1->AddPort(105, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(107);
                sub2->AddTran(201, 101, 106, NODE_vss);
                sub2->AddTran(202, 102, 106, NODE_vss);
                sub2->AddTran(203, 103, 107, 106);
                sub2->AddTran(204, 104, 107, 106);
                sub2->AddTran(205, 105, 107, NODE_vss);
        }
        return "~((1+2).(3+4)+5)";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = ~((%s|%s)&%s|%s);\n",
                    sub1->GetMate(105), sub1->GetRoot(101), sub1->GetRoot(102), sub1->GetRoot(103), sub1->GetRoot(104));
                break;
            case CMD_PORTS:
                sub1->AddPort(105, TT_OUT);
                sub1->AddPort(101, TT_IN);
                sub1->AddPort(102, TT_IN);
                sub1->AddPort(103, TT_IN);
                sub1->AddPort(104, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(105);
                sub2->AddTran(201, 101, 106, NODE_vss);
                sub2->AddTran(202, 102, 106, NODE_vss);
                sub2->AddTran(203, 103, 105, 106);
                sub2->AddTran(204, 104, 105, NODE_vss);
        }
        return "~((1+2).3+4)";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = ~((%s|%s)&%s);\n",
                    sub1->GetMate(104), sub1->GetRoot(101), sub1->GetRoot(102), sub1->GetRoot(103));
                break;
            case CMD_PORTS:
                sub1->AddPort(104, TT_OUT);
                sub1->AddPort(101, TT_IN);
                sub1->AddPort(102, TT_IN);
                sub1->AddPort(103, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(104);
                sub2->AddTran(201, 101, 105, NODE_vss);
                sub2->AddTran(202, 102, 105, NODE_vss);
                sub2->AddTran(203, 103, 104, 105);
        }
        return "~((1+2).3)";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = ~((%s|%s)&%s);\n",
                    sub1->GetMate(104), sub1->GetRoot(101), sub1->GetRoot(102), sub1->GetRoot(103));
                break;
            case CMD_PORTS:
                sub1->AddPort(104, TT_OUT);
                sub1->AddPort(101, TT_IN);
                sub1->AddPort(102, TT_IN);
                sub1->AddPort(103, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(104);
                sub2->AddTran(201, 101, 104, 105);
                sub2->AddTran(202, 102, 104, 105);
                sub2->AddTran(203, 103, 105, NODE_vss);
        }
        return "~((1+2).3)";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = ~(%s&(%s|%s&%s)|%s&%s);\n",
                    sub1->GetMate(107), sub1->GetRoot(105), sub1->GetRoot(102), sub1->GetRoot(101), sub1->GetRoot(103), sub1->GetRoot(104), sub1->GetRoot(106));
                break;
            case CMD_PORTS:
                sub1->AddPort(107, TT_OUT);
                sub1->AddPort(101, TT_IN);
                sub1->AddPort(102, TT_IN);
                sub1->AddPort(103, TT_IN);
                sub1->AddPort(104, TT_IN);
                sub1->AddPort(105, TT_IN);
                sub1->AddPort(106, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(107);
                sub2->AddTran(201, 101, 110, NODE_vss);
                sub2->AddTran(202, 102, 108, NODE_vss);
                sub2->AddTran(203, 103, 108, 110);
                sub2->AddTran(204, 104, 109, NODE_vss);
                sub2->AddTran(205, 105, 107, 108);
                sub2->AddTran(206, 106, 107, 109);
        }
        return "~(5.(2+1.3) + 4.6)";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                if (sub1->GetKeep(102)) fprintf(fp, "assign o[%d] = ~%s;\n",
                    sub1->GetMate(102), sub1->GetRoot(101));
                break;
            case CMD_UNARY:
                UnaryInv(sub1->GetMate(102), sub1->GetMate(101));
                break;
            case CMD_PORTS:
                sub1->AddPort(102, TT_OUT);
                sub1->AddPort(101, TT_PAD);
                break;
            case CMD_LOGIC:
                sub2->AddPull(101);
                sub2->AddPull(102);
                sub2->AddTran(201, 101, 102, NODE_vss);
                sub2->AddTran(202, NODE_vss, NODE_vss, 101);
        }
        return "RDY and SO";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                if (sub1->GetKeep(102)) fprintf(fp, "assign o[%d] = ~%s;\n",
                    sub1->GetMate(102), sub1->GetRoot(101));
                break;
            case CMD_UNARY:
                UnaryInv(sub1->GetMate(102), sub1->GetMate(101));
                break;
            case CMD_PORTS:
                sub1->AddPort(102, TT_OUT);
                sub1->AddPort(101, TT_PAD);
                break;
            case CMD_LOGIC:
                sub2->AddPull(102);
                sub2->AddTran(201, 101, 102, NODE_vss);
                sub2->AddTran(202, NODE_vss, NODE_vss, 101);
        }
        return "IRQ, NMI and RES";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = ~(%s|i[%d]);\n",
                    sub1->GetMate(101), sub1->GetRoot(102), NODE_cp2);
                break;
            case CMD_PORTS:
                sub1->AddPort(101, TT_OUT);
                sub1->AddPort(102, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(101);
                sub2->AddTran(201, NODE_cp2, 101, NODE_vss);
                sub2->AddTran(202, 102, 101, NODE_vss);
        }
        return "~(2+cp2)";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                if (sub1->GetKeep(101)) fprintf(fp, "assign o[%d] = ~i[%d]; // ~cp2\n",
                    sub1->GetMate(101), NODE_cp2);
                break;
            case CMD_UNARY:
                UnaryInv(sub1->GetMate(101), NODE_cp2);
                break;
            case CMD_PORTS:
                sub1->AddPort(101, TT_OUT);
                break;
            case CMD_LOGIC:
                sub2->AddPull(101);
                sub2->AddTran(201, NODE_cp2, 101, NODE_vss);
        }
        return "~cp2";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = ~((%s|%s|%s)&%s);\n",
                    sub1->GetMate(105), sub1->GetRoot(101), sub1->GetRoot(102), sub1->GetRoot(103), sub1->GetRoot(104));
                break;
            case CMD_PORTS:
                sub1->AddPort(105, TT_OUT);
                sub1->AddPort(101, TT_IN);
                sub1->AddPort(102, TT_IN);
                sub1->AddPort(103, TT_IN);
                sub1->AddPort(104, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(105);
                sub2->AddTran(201, 101, 106, NODE_vss);
                sub2->AddTran(202, 102, 106, NODE_vss);
                sub2->AddTran(203, 103, 106, NODE_vss);
                sub2->AddTran(204, 104, 105, 106);
        }
        return "~((1+2+3).4)";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = ~(%s|%s);\n", sub1->GetMate(104), sub1->GetRoot(102), sub1->GetRoot(103));
                fprintf(fp, "assign o[%d] = ~(%s|%s);\n", sub1->GetMate(101), sub1->GetRoot(102), sub1->GetRoot(103));
                break;
            case CMD_PORTS:
                sub1->AddPort(104, TT_OUT);
                sub1->AddPort(101, TT_OUT);
                sub1->AddPort(102, TT_IN);
                sub1->AddPort(103, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(101);
                sub2->AddTran(201, 101, 104, NODE_vcc);
                sub2->AddTran(202, 102, 104, NODE_vss);
                sub2->AddTran(203, 103, 104, NODE_vss);
                sub2->AddTran(204, 102, 101, NODE_vss);
                sub2->AddTran(205, 103, 101, NODE_vss);
        }
        return "clock1";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                fprintf(fp, "assign o[%d] = (%s^%s) & %s;\n",
                    sub1->GetMate(100430), sub1->GetRoot(100771), sub1->GetRoot(100206), sub1->GetRoot(100899));
                break;
            case CMD_PORTS:
                sub1->AddPort(100430, TT_OUT);
                sub1->AddPort(100771, TT_IN);
                sub1->AddPort(100206, TT_IN);
                sub1->AddPort(100899, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddPull(100465);
                sub2->AddPull(101446);
                sub2->AddPull(100850);
                sub2->AddTran(202669, 100206, 100465, NODE_vss);
                sub2->AddTran(202882, 100771, 101446, NODE_vss);
                sub2->AddTran(202144, 100850, 101446, NODE_vss);
                sub2->AddTran(202145, 100850, 100430, NODE_vss);
                sub2->AddTran(200886, 100899, 100850, NODE_vss);
                sub2->AddTran(201273, 101446, 100430, 100206);
                sub2->AddTran(202883, 100771, 100430, 100465);
        }
        return "short-circuit-branch-add";
    }
    if (ord==idx++) {
        std::map<int,int>::iterator it;
        char H1x1_d[10], H1x1_s[10], ADL0_s[10];

        switch (cmd) {
            case CMD_VERILOG:
                it = Map_H1x1.find(sub1->GetMate(501));
                GetRoot(H1x1_d, it!=Map_H1x1.end()? it->second : NODE_vss);
                GetRoot(H1x1_s, it!=Map_H1x1.end()? NODE_H1x1  : NODE_vss);

                it = Map_0ADL.find(sub1->GetMate(502));
                GetRoot(ADL0_s, it!=Map_0ADL.end()? it->second : NODE_vss);

                fprintf(fp, "wire sb_local_%d = %s|%s|%s|%s|%s;\n",
                    sub1->GetMate(500),
                    sub1->GetRoot(300), sub1->GetRoot(302), sub1->GetRoot(304), sub1->GetRoot(319), sub1->GetRoot(324));

                fprintf(fp, "wire idb_local_%d = %s|%s|%s|i[%d]&%s|%s;\n",
                    sub1->GetMate(501),
                    sub1->GetRoot(326), sub1->GetRoot(333), sub1->GetRoot(337), NODE_cp1, sub1->GetRoot(343), H1x1_s);

                fprintf(fp, "wire adh_local_%d = %s|%s|i[%d]&%s;\n",
                    sub1->GetMate(503),
                    sub1->GetRoot(328), sub1->GetRoot(332), NODE_cp1, sub1->GetRoot(342));

                fprintf(fp, "wire db2sb_%d = idb_local_%d & %s;\n",
                    sub1->GetMate(500), sub1->GetMate(501), sub1->GetRoot(325));

                fprintf(fp, "wire sb2db_%d = sb_local_%d & %s;\n",
                    sub1->GetMate(501), sub1->GetMate(500), sub1->GetRoot(325));

                fprintf(fp, "wire adh2sb_%d = adh_local_%d & %s;\n",
                    sub1->GetMate(500), sub1->GetMate(503), sub1->GetRoot(327));

                fprintf(fp, "wire sb2adh_%d = sb_local_%d & %s;\n",
                    sub1->GetMate(503), sub1->GetMate(500), sub1->GetRoot(327));

                fprintf(fp, "MUX #(8) mux_sb_%d (.o(o[%d]), .i(%s), "
                    ".s({i[%d],%s,%s,%s,%s,%s,db2sb_%d,adh2sb_%d}), "
                    ".d({i[%d],%s,%s,%s,%s,%s,%s,%s}));\n",
                    sub1->GetMate(500), sub1->GetMate(500), sub1->GetRoot(500),
                    NODE_cp2, sub1->GetRoot(300), sub1->GetRoot(302), sub1->GetRoot(304), sub1->GetRoot(319), sub1->GetRoot(324), sub1->GetMate(500), sub1->GetMate(500),
                    NODE_vcc, sub1->GetRoot(202), sub1->GetRoot(201), sub1->GetRoot(204), sub1->GetRoot(207), sub1->GetRoot(203), sub1->GetRoot(501), sub1->GetRoot(503));

                fprintf(fp, "MUX #(7) mux_idb_%d (.o(o[%d]), .i(%s), "
                    ".s({i[%d],%s,%s,%s,i[%d]&%s,sb2db_%d,%s}), "
                    ".d({i[%d],%s,%s,%s,%s,%s,%s}));\n",
                    sub1->GetMate(501), sub1->GetMate(501), sub1->GetRoot(501),
                    NODE_cp2, sub1->GetRoot(326), sub1->GetRoot(333), sub1->GetRoot(337), NODE_cp1, sub1->GetRoot(343), sub1->GetMate(501), H1x1_s,
                    NODE_vcc, sub1->GetRoot(203), sub1->GetRoot(206), sub1->GetRoot(205),           sub1->GetRoot(200), sub1->GetRoot(500), H1x1_d);

                fprintf(fp, "MUX #(6) mux_adl_%d (.o(o[%d]), .i(%s), "
                    ".s({i[%d],%s,%s,%s,i[%d]&%s,%s}), "
                    ".d({i[%d],%s,%s,%s,%s,i[%d]}));\n",
                    sub1->GetMate(502), sub1->GetMate(502), sub1->GetRoot(502),
                    NODE_cp2, sub1->GetRoot(305), sub1->GetRoot(321), sub1->GetRoot(338), NODE_cp1, sub1->GetRoot(341), ADL0_s,
                    NODE_vcc, sub1->GetRoot(204), sub1->GetRoot(207), sub1->GetRoot(205),           sub1->GetRoot(200), NODE_vss);

                fprintf(fp, "MUX #(5) mux_adh_%d (.o(o[%d]), .i(%s), "
                    ".s({i[%d],%s,%s,i[%d]&%s, sb2adh_%d}), "
                    ".d({i[%d],i[%d],%s,%s,%s}));\n",
                    sub1->GetMate(503), sub1->GetMate(503), sub1->GetRoot(503),
                    NODE_cp2, sub1->GetRoot(328), sub1->GetRoot(332), NODE_cp1, sub1->GetRoot(342), sub1->GetMate(503),
                    NODE_vcc, NODE_vss,           sub1->GetRoot(206),           sub1->GetRoot(200), sub1->GetRoot(500));

                fprintf(fp, "MUX #(2) mux_pcl_%d (.o(o[%d]), .i(%s), "
                    ".s({%s,%s}), "
                    ".d({%s,%s}));\n",
                    sub1->GetMate(100), sub1->GetMate(100), sub1->GetRoot(100), sub1->GetRoot(339), sub1->GetRoot(340), sub1->GetRoot(205), sub1->GetRoot(502));

                fprintf(fp, "MUX #(2) mux_pch_%d (.o(o[%d]), .i(%s), "
                    ".s({%s,%s}), "
                    ".d({%s,%s}));\n",
                    sub1->GetMate(101), sub1->GetMate(101), sub1->GetRoot(101), sub1->GetRoot(330), sub1->GetRoot(331), sub1->GetRoot(503), sub1->GetRoot(206));
                break;
            case CMD_PORTS:
                sub1->AddPort(500, TT_OUT);      // sb
                sub1->AddPort(501, TT_OUT);      // idb
                sub1->AddPort(502, TT_OUT);      // adl
                sub1->AddPort(503, TT_OUT);      // adh
                sub1->AddPort(100, TT_OUT);      // ->pcl
                sub1->AddPort(101, TT_OUT);      // ->pch
                sub1->AddPort(200, TT_IN);       // idl->
                sub1->AddPort(201, TT_IN);       // x->
                sub1->AddPort(202, TT_IN);       // y->
                sub1->AddPort(203, TT_IN);       // a->
                sub1->AddPort(204, TT_IN);       // s->
                sub1->AddPort(205, TT_IN);       // pcl->
                sub1->AddPort(206, TT_IN);       // pch->
                sub1->AddPort(207, TT_IN);       // alu->
                sub1->AddPort(300, TT_IN,  801); // dpc0_YSB
                sub1->AddPort(302, TT_IN, 1263); // dpc2_XSB
                sub1->AddPort(304, TT_IN, 1700); // dpc4_SSB
                sub1->AddPort(305, TT_IN);       // dpc5_SADL
                sub1->AddPort(319, TT_IN);       // dpc19_ADDSB7, dpc20_ADDSB06
                sub1->AddPort(321, TT_IN);       // dpc21_ADDADL
                sub1->AddPort(324, TT_IN, 1698); // dpc24_ACSB
                sub1->AddPort(325, TT_IN);       // dpc25_SBDB
                sub1->AddPort(326, TT_IN);       // dpc26_ACDB
                sub1->AddPort(327, TT_IN);       // dpc27_SBADH
                sub1->AddPort(328, TT_IN);       // dpc28_0ADH0, dpc29_0ADH17
                sub1->AddPort(330, TT_IN);       // dpc30_ADHPCH
                sub1->AddPort(331, TT_IN);       // dpc31_PCHPCH
                sub1->AddPort(332, TT_IN);       // dpc32_PCHADH
                sub1->AddPort(333, TT_IN);       // dpc33_PCHDB
                sub1->AddPort(337, TT_IN);       // dpc37_PCLDB
                sub1->AddPort(338, TT_IN);       // dpc38_PCLADL
                sub1->AddPort(339, TT_IN);       // dpc39_PCLPCL
                sub1->AddPort(340, TT_IN);       // dpc40_ADLPCL
                sub1->AddPort(341, TT_IN);       // dpc41_DL/ADL
                sub1->AddPort(342, TT_IN);       // dpc42_DL/ADH
                sub1->AddPort(343, TT_IN);       // dpc43_DL/DB
                break;
            case CMD_LOGIC:
                sub2->AddTran(400, 300, 202, 500);
                sub2->AddTran(402, 302, 201, 500);
                sub2->AddTran(404, 304, 204, 500);
                sub2->AddTran(405, 305, 204, 502);
                sub2->AddTran(419, 319, 207, 500);
                sub2->AddTran(421, 321, 207, 502);
                sub2->AddTran(424, 324, 203, 500);
                sub2->AddTran(425, 325, 500, 501);
                sub2->AddTran(426, 326, 203, 501);
                sub2->AddTran(427, 327, 500, 503);
                sub2->AddTran(428, 328, 503, NODE_vss);
                sub2->AddTran(430, 330, 503, 101);
                sub2->AddTran(431, 331, 206, 101);
                sub2->AddTran(432, 332, 206, 503);
                sub2->AddTran(433, 333, 206, 501);
                sub2->AddTran(437, 337, 205, 501);
                sub2->AddTran(438, 338, 205, 502);
                sub2->AddTran(439, 339, 205, 100);
                sub2->AddTran(440, 340, 502, 100);
                sub2->AddTran(441, 341, 600, 502);
                sub2->AddTran(442, 342, 600, 503);
                sub2->AddTran(443, 343, 600, 501);
                sub2->AddTran(700, NODE_cp1, 200, 600);
                sub2->AddTran(800, NODE_cp2, 500, NODE_vcc);
                sub2->AddTran(801, NODE_cp2, 501, NODE_vcc);
                sub2->AddTran(802, NODE_cp2, 502, NODE_vcc);
                sub2->AddTran(803, NODE_cp2, 503, NODE_vcc);
        }
        return "Datapath bitslice";
    }
    if (ord<idx+NumGates) {
        const char *p = Gates[ord-idx];
        switch (cmd) {
            case CMD_VERILOG:
                if (sub1->GetKeep(300)==0) break;
                fprintf(fp, "assign o[%d] = ~(", sub1->GetMate(300));
                GateVerilog(sub1, p+1, fp);
                fprintf(fp, ");\n");
                break;
            case CMD_UNARY:
                if (p[1]=='1' && p[2]==0) UnaryInv(sub1->GetMate(300), sub1->GetMate(100));
                break;
            case CMD_PORTS:
                sub1->AddPort(300, TT_OUT);
                GatePorts(sub1, p+1);
                break;
            case CMD_LOGIC:
                GateLogic(sub2, p+1);
        }
        return p;
    }
    else
        idx+=NumGates;

    if (ord<idx+10) {
        const char *p = Gates[ord-idx+NumGates-10];
        switch (cmd) {
            case CMD_VERILOG:
                if (sub1->GetKeep(300)==0) break;
                fprintf(fp, "assign o[%d] = ", sub1->GetMate(300));
                GateVerilog(sub1, p+1, fp);
                fprintf(fp, ";\n");
                break;
            case CMD_UNARY:
                if (p[1]=='1' && p[2]==0) UnaryBuf(sub1->GetMate(300), sub1->GetMate(100));
                break;
            case CMD_PORTS:
                sub1->AddPort(300, TT_OUT);
                GatePorts(sub1, p+1);
                break;
            case CMD_LOGIC:
                GateLogic((GRAPH*)sub, p+1, ord-10, idx-1);
        }
        return p+1;
    }
    else
        idx+=10;

    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                // See mux_adl_*
                break;
            case CMD_PORTS:
                sub1->AddPort(101, TT_IN);
                sub1->AddPort(102, TT_IN);
                break;
            case CMD_LOGIC:
                sub2->AddTran(201, 101, 102, NODE_vss);
                break;
            case CMD_0ADL_H1x1:
                Map_0ADL[sub1->GetMate(102)] = sub1->GetMate(101);
        }
        return "0ADL";
    }
    if (ord==idx++) {
        switch (cmd) {
            case CMD_VERILOG:
                // See mux_idb_*
                    break;
            case CMD_PORTS:
                sub1->AddPort(101, TT_IN);
                sub1->AddPort(102, TT_IN);
                sub1->AddPort(103, TT_IN);
                break;
            case CMD_LOGIC:
                // Does not match latches because their gates are special nodes
                sub2->AddTran(201, 101, 102, 103);
                break;
            case CMD_0ADL_H1x1:
                if (sub1->GetMate(101) != NODE_H1x1) error(20);
                Map_H1x1[sub1->GetMate(102)] = sub1->GetMate(103);
                Map_H1x1[sub1->GetMate(103)] = sub1->GetMate(102);
        }
        return "H1x1";
    }

    return 0;
}
