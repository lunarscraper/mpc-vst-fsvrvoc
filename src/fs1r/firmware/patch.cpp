// fs1r/firmware/patch.cpp - voice, performance and part data as the firmware decodes it. KNOWN.
#include "fs1r/internal.h"

// The DX7 ACED buffer: a voice edit arrives as VCED then ACED, so the ACED half is held here
// until the VCED that completes it turns up.
uint8_t g_aced[49];
bool g_acedValid = false;

void decode_voice(Voice& V) {
    const uint8_t* b = V.raw;
    memcpy(V.name, b, 10); V.name[10] = 0; V.cat = b[0x0E];
    V.lfo1wave = std::min<int>(b[0x10], 5); V.lfo1speed = b[0x11]; V.lfo1delay = b[0x12]; V.lfo1sync = b[0x13] & 1;
    V.pmd = b[0x15]; V.amd = b[0x16]; V.fmd = b[0x17];
    V.lfo2wave = std::min<int>(b[0x18], 5); V.lfo2speed = b[0x19]; V.lfo2phase = b[0x1C] & 3; V.lfo2sync = b[0x1D] & 1;
    V.fltType = std::min<int>(b[0x54], 5); V.fltReso = (b[0x55] & 0x7F) - 16; V.fltResoVel = (b[0x56] & 0xF) - 7;
    V.fltCut = b[0x57]; V.fltEgVel = (b[0x58] & 0xF) - 7; V.fltLfo1 = b[0x59]; V.fltLfo2 = b[0x5A];
    V.fltKsDepth = b[0x5B] - 64; V.fltKsPoint = b[0x5C]; V.fltInGain = (b[0x5D] & 0x1F) - 12; V.fegDepth = b[0x64] - 64;
    V.fltL[0] = b[0x66]; V.fltL[1] = b[0x67]; V.fltL[2] = b[0x68]; V.fltL[3] = b[0x65];   // L1 L2 L3, L4 is the start level
    for (int i = 0; i < 4; i++) V.fltT[i] = b[0x69 + i];
    V.fltAtkVel = b[0x6E] >> 3 & 7; V.fltTscale = b[0x6E] & 7;
    V.noteshift = (b[0x1E] & 0x3F) - 24;
    V.pegL[0] = b[0x1F]; V.pegL[1] = b[0x20]; V.pegL[2] = b[0x21]; V.pegL[4] = b[0x22]; V.pegL[3] = b[0x3E];
    for (int i = 0; i < 4; i++) V.pegT[i] = b[0x23 + i];
    V.pegVel = b[0x27] & 7; V.pegRange = b[0x3B] & 3; V.pegTscale = b[0x3C] & 7;
    V.fseqV = (b[0x28] & 1) << 7 | (b[0x29] & 0x7F); V.fseqU = (b[0x2A] & 1) << 7 | (b[0x2B] & 0x7F);
    V.alg = std::min<int>(b[0x2C], 87); for (int i = 0; i < 8; i++) V.corr[i] = b[0x2D + i] & 0xF; V.fb = b[0x3D] & 7;
    for (int o = 0; o < 8; o++) {
        const uint8_t* p = b + 112 + o * 62; OpV& v = V.v[o];
        v.keysync = p[0] >> 6 & 1; v.transpose = (p[0] & 0x3F) - 24;
        v.coarse = p[1] & 31; v.fine = p[2]; v.notescale = p[3];
        v.bwbias = (p[4] >> 3 & 0xF); v.form = p[4] & 7;
        v.fixed = p[5] >> 6 & 1; v.skirt = p[5] >> 3 & 7; v.fseqtrk = p[5] & 7;
        v.bw = p[6]; v.detune = p[7];
        v.fegInit = p[8] - 50; v.fegAtt = p[9] - 50; v.fegAttT = p[10]; v.fegDecT = p[11];
        for (int i = 0; i < 4; i++) { v.L[i] = p[12 + i]; v.T[i] = p[16 + i]; }
        v.hold = p[20]; v.tscale = p[21] & 7; v.level = p[22]; v.bp = p[23]; v.ld = p[24]; v.rd = p[25]; v.lc = p[26] & 3; v.rc = p[27] & 3;
        v.fbias = (p[31] >> 3 & 0xF); v.pms = p[31] & 7;
        v.fms = p[32] >> 4 & 7; v.fmsb = p[32] & 0xF;
        v.ams = p[33] >> 4 & 7; v.amsb = p[33] & 0xF; v.egbias = (p[34] & 0xF) - 7;
        const uint8_t* q = p + 35; OpU& u = V.u[o];
        u.transpose = (q[0] & 0x3F) - 24; u.mode = q[1] >> 5 & 3; u.coarse = q[1] & 0x1F; u.fine = q[2]; u.notescale = q[3];
        u.bw = q[4]; u.bwbias = q[5] & 0xF; u.res = q[6] >> 3 & 7; u.skirt = q[6] & 7;
        u.fegInit = q[7] - 50; u.fegAtt = q[8] - 50; u.fegAttT = q[9]; u.fegDecT = q[10];
        u.level = q[11]; u.lks = (q[12] & 0xF) - 7;
        for (int i = 0; i < 4; i++) { u.L[i] = q[13 + i]; u.T[i] = q[17 + i]; }
        u.hold = q[21]; u.tscale = q[22] & 7;
        u.fbias = (q[23] & 0xF); u.fms = q[24] >> 4 & 7; u.fmsb = q[24] & 0xF;
        u.ams = q[25] >> 4 & 7; u.amsb = q[25] & 0xF; u.egbias = (q[26] & 0xF) - 7;
    }
}

void init_blank_voice(uint8_t* b) {
    memset(b, 0, 608); memcpy(b, "Init      ", 10);
    b[0x1E] = 24; b[0x1F] = b[0x20] = b[0x21] = b[0x22] = b[0x3E] = 50; b[0x2C] = 0; b[0x3D] = 0;
    b[0x55] = 16; b[0x56] = b[0x58] = 7; b[0x57] = 127; b[0x5B] = b[0x5C] = 64; b[0x5D] = 12; b[0x64] = 64;
    b[0x65] = b[0x66] = b[0x67] = b[0x68] = 50;
    for (int o = 0; o < 8; o++) {
        uint8_t* p = b + 112 + o * 62;
        p[0] = 24; p[1] = 1; p[4] = 7 << 3; p[5] = (uint8_t)o; p[7] = 15; p[8] = p[9] = 50;
        p[12] = 99; p[13] = 99; p[14] = 99; p[15] = 0; p[16] = 0; p[17] = 50; p[18] = 50; p[19] = 40;
        p[22] = 0; p[23] = 39; p[31] = 7 << 3; p[32] = 7; p[33] = 7; p[34] = 7;
        uint8_t* q = p + 35; q[0] = 24; q[7] = q[8] = 50; q[12] = 7; q[23] = 7; q[24] = 7; q[25] = 7; q[26] = 7;
    }
}

void init_default_voice(Voice& V) {
    init_blank_voice(V.raw); uint8_t* b = V.raw;
    memcpy(b, "InitEP    ", 10); b[0x2C] = 12; b[0x3D] = 5;
    auto op = [&](int o) { return b + 112 + o * 62; };
    op(2)[22] = 70; op(2)[1] = 14; op(2)[13] = 0; op(2)[17] = 30;
    op(3)[22] = 99;
    op(4)[22] = 78; op(4)[1] = 1;  op(4)[13] = 60; op(4)[17] = 55;
    op(5)[22] = 95;
    decode_voice(V);
}

void apply_aced(uint8_t* out) {
    if (!g_acedValid) return;
    for (int j = 0; j < 6; j++) {                 // ACED 6..11 are OP6..OP1, the same order as the VCED
        uint8_t* p = out + 112 + (2 + j) * 62;
        p[33] = (uint8_t)(((g_aced[6 + j] & 7) << 4) | (p[33] & 0x0F));
    }
    out[0x3B] = g_aced[0x0C] & 3;                 // pitch EG range
    out[0x27] = g_aced[0x0E] ? 7 : 0;             // pitch EG velocity switch
    out[0x3C] = g_aced[0x26] & 7;                 // pitch EG rate scaling
    g_acedValid = false;
}

void convert_dx7(const uint8_t* v, uint8_t* out) {
    // FUN_00035918 (common) and FUN_00035E96 (one operator), byte for byte. The common block is a list of
    // 16-bit words the firmware then packs; the per-algorithm row DX7MAP[alg] carries the FS1R algorithm,
    // the operator slot each DX operator lands on, +2 on the output level where the row says so, and
    // the carrier level corrections. Checked against the demo's own converted bulks (FullTine 1, DX-Acrd 4)
    // on every byte the demo did not edit afterwards.
    memset(out, 0, 608);
    for (int i = 0; i < 10; i++) { uint8_t c = v[145 + i]; out[i] = (c < 0x20 || c > 0x7D) ? 0x20 : c; }
    const unsigned short* row = DX7MAP + 33 * (v[134] & 31);
    out[0x10] = v[142]; out[0x11] = v[137]; out[0x12] = v[138]; out[0x13] = v[141]; out[0x15] = v[139] >> 1; out[0x16] = v[140];
    out[0x18] = v[142]; out[0x19] = v[137];                                       // LFO2 takes LFO1's wave and speed
    out[0x1E] = v[144];
    auto l100 = [](int x) { return (uint8_t)(x == 99 ? 100 : x); };            // PEG levels: 99 becomes 100
    out[0x1F] = l100(v[133]); out[0x20] = l100(v[130]); out[0x21] = l100(v[131]); out[0x22] = l100(v[133]); out[0x3E] = l100(v[132]);
    for (int i = 0; i < 4; i++) out[0x23 + i] = (uint8_t)(99 - v[126 + i]);
    out[0x2C] = (uint8_t)row[16]; for (int o = 0; o < 8; o++) out[0x2D + o] = (uint8_t)((row[2 * o + 1] >> 3) & 0xF);
    out[0x3D] = v[135];
    out[0x40] = (uint8_t)(1 << 4 | DX7CTRL[2 * (v[134] & 31)]);     out[0x45] = 74;  // formant control 1: output, +10, on the algorithm's op
    out[0x46] = out[0x47] = out[0x48] = out[0x49] = 64;
    out[0x4A] = (uint8_t)(1 << 4 | DX7CTRL[2 * (v[134] & 31) + 1]); out[0x4F] = 71;  // FM control 1: output, +7
    out[0x50] = out[0x51] = out[0x52] = out[0x53] = 64;
    out[0x55] = 0x24; out[0x56] = 10; out[0x57] = 85; out[0x58] = 7; out[0x5B] = 69; out[0x5C] = 60; out[0x5D] = 12;   // the filter block's defaults
    out[0x64] = 64; out[0x65] = 50; out[0x66] = 100; out[0x67] = out[0x68] = 75; out[0x69] = out[0x6A] = out[0x6B] = 30; out[0x6C] = 99;
    // the two unused slots take the template at 0x38A914 (35 bytes), with their Fseq track = the slot
    // (both converted bulks carry break point 60 and the call index 6/7 in byte 5, where the ROM template has 39 and 0)
    for (int o = 0, j = 6; o < 8; o++) { uint8_t* p = out + 112 + o * 62; memcpy(p, DX7OPTEMPLATE, 35); p[23] = 60; memcpy(p + 35, DX7UOP + 27 * o, 27);
        bool used = false; for (int k = 0; k < 6; k++) if (row[25 + k] == o) used = true;
        if (!used) p[5] = (uint8_t)(j++); }
    for (int j = 0; j < 6; j++) {                     // j = 0 is DX operator 6
        const uint8_t* d = v + j * 21; uint8_t* p = out + 112 + row[25 + j] * 62;   // slot: word 25 + j (0x389FBE)
        int ol = d[16] + (row[17 + j] == 1 ? 2 : 0), rs = d[13];
        int att = rs * 1386 / 504 + (99 - ol) * 4 / 10 + 6, dec = rs * 1386 / 504 + (99 - ol) * 2 / 10;
        int coarse = d[18], fine = d[19], fixed = d[17] & 1;
        if (fixed) {
            double hz = pow(10.0, coarse % 4) * pow(10.0, fine / 100.0);        // FUN_00035658, software float, INFERRED
            double x = log2(hz / 440.0) + 16; int c = (int)floor(x); int f = (int)lround((x - c) * 128);
            if (f > 127) { f = 0; c++; }
            coarse = clampi(c, 0, 31); fine = clampi(f, 0, 127);
        }
        p[0] = (uint8_t)((v[136] & 1) << 6 | 24); p[1] = (uint8_t)coarse; p[2] = (uint8_t)fine; p[3] = 0;
        p[4] = 7 << 3; p[5] = (uint8_t)(fixed << 6 | 0 << 3 | j);              // bw bias 0, sine, skirt 0, Fseq track = DX index
        p[6] = 0; p[7] = (uint8_t)(d[20] + 8); p[8] = p[9] = 50; p[10] = p[11] = 0;
        for (int i = 0; i < 4; i++) {
            int r = d[i];
            int t = i == 0 ? 99 - DX7RATE_A[r] - r - att : 99 - DX7RATE_B[r] - r - dec;
            p[16 + i] = (uint8_t)clampi(t, 0, 99); p[12 + i] = d[4 + i];
        }
        p[20] = 0; p[21] = (uint8_t)rs; p[22] = (uint8_t)ol;
        p[23] = d[8]; p[24] = d[9]; p[25] = d[10]; p[26] = d[11]; p[27] = d[12]; p[28] = p[29] = p[30] = 0;
        p[31] = (uint8_t)(7 << 3 | v[143]); p[32] = 7;
        p[33] = (uint8_t)(d[14] << 4 | (d[15] + 7)); p[34] = (uint8_t)(d[14] + 7);   // amp mod sense, velocity sense; EG bias sense = ams
    }
    apply_aced(out);
}

int bank_perf_index(int lsb, int prog) {
    int base = lsb == 0x42 ? 128 : lsb == 0x43 ? 256 : 0;   // PrB, PrC, else Int and PrA
    return clampi(base + (prog & 0x7F), 0, 383);
}

void Part::reset_ctl() {
    bend = 0; expr = 254; memset(src, 0, sizeof src); sustain = false; rpnM = rpnL = 127;
}

void init_perf(Perf& P) {
    memset(P.c, 0, 80); memset(P.fx, 0, 112); memcpy(P.c, "Init        ", 12); P.c[0x10] = 127; P.c[0x11] = 64; P.c[0x12] = 24;
    P.c[0x18] = 7; P.c[0x19] = 104;   // Fseq speed ratio 1000 = 100 %
    P.c[0x21] = 2;
    for (int i = 0; i < 4; i++) {
        Part& pt = P.part[i]; memset(pt.p, 0, 52);
        pt.p[0] = 8; pt.p[1] = 1; pt.p[3] = 0x7F; pt.p[4] = i ? 0x7F : 0x10; pt.p[5] = 1; pt.p[8] = 24; pt.p[9] = 64; pt.p[10] = 64; pt.p[11] = 127; pt.p[12] = 64; pt.p[13] = 64;
        pt.p[14] = 64; pt.p[15] = 0; pt.p[16] = 127; pt.p[17] = 127; pt.p[0x15] = pt.p[0x16] = pt.p[0x17] = 64;
        for (int k = 0x18; k <= 0x23; k++) pt.p[k] = 64;
        pt.p[0x28] = 50; pt.p[0x2E] = pt.p[0x2F] = 64;   // pan scaling +0, LFO2 rate / filter mod offsets 0
        pt.p[0x25] = 0; pt.p[0x26] = 0x40 + 2; pt.p[0x27] = 0x40 - 2; pt.p[0x2A] = 1; pt.p[0x2B] = 127; pt.p[0x2D] = 1; pt.p[0x2E] = pt.p[0x2F] = 64;
        init_default_voice(pt.voice);
    }
    memcpy(P.name, P.c, 12); P.name[12] = 0;
}

