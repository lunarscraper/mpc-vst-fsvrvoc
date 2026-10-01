// fs1r/firmware/controllers.cpp - the eight controller sets and the 48 destinations. KNOWN: rewritten from the disassembly.
#include "fs1r/internal.h"

int Synth::scale_f032(int v, int d, int base) {
    int r = clampi((ctrl_bias(v) * ctrl_bias(d) * 2) >> 6, -128, 32767);
    if (r > 0) r = std::min(r - 1, 127);
    return clampi(base + r, 0, 127);
}

int Synth::ctrl_set_sum(int part, int set) const {
    int bits = perf.c[0x30 + 2 * set] << 7 | perf.c[0x31 + 2 * set], sum = 0;
    for (int b = 0; b < 14; b++) if (bits & (1 << b)) sum = clampi(sum + perf.part[part].src[b], -128, 127);
    return sum;
}

bool Synth::ctrl_set_active(int part, int set, int dest) const {
    int d = perf.c[0x40 + set] & 0x3F;
    if (!d || d >= 0x30 || d != dest) return false;
    if (dest >= 17 && dest <= 45 && !(perf.c[0x28 + set] & (1 << part))) return false;
    return true;
}

int Synth::ctrl_offset(int part, int dest) const {
    for (int s = 7; s >= 0; s--)
        if (ctrl_set_active(part, s, dest))
            return scale_f9c(ctrl_set_sum(part, s), perf.c[0x48 + s] - 64);
    return 0;
}

int Synth::ctrl_part(int part, int dest, int stored) const {
    bool wide = dest == 17 || dest == 19 || dest == 20;     // volume and the two sends use FUN_00017032
    for (int s = 7; s >= 0; s--)
        if (ctrl_set_active(part, s, dest)) {
            int v = ctrl_set_sum(part, s), d = perf.c[0x48 + s] - 64;
            return wide ? scale_f032(v, d, stored) : scale_f58(v, d, stored);
        }
    return clampi(stored, 0, 127);
}

int Synth::ctrl_pitch_bias(int part) const {
    int w = 0;
    for (int s = 0; s < 8; s++) {
        if (!ctrl_set_active(part, s, 34)) continue;
        int v = ctrl_set_sum(part, s), d = clampi(perf.c[0x48 + s] - 64, -24, 24);
        int b = BENDTAB[std::abs(d)];
        if (d < 0) b = -b;
        w += (b * (v == 0x7F ? 0x80 : v)) >> 7;
    }
    return w;
}

int Synth::ctrl_eg_bias(int part) const {
    static const short DEPTHTERM[32] = {127, 119, 115, 111, 107, 103, 99, 95, 91, 87, 83, 79, 75, 71, 67, 63,
                                        59, 55, 51, 47, 43, 39, 35, 31, 27, 23, 19, 15, 11, 7, 3, 0};
    // No set assigned to 35 means no bias at all. 255 would be full bias, which is what an assigned
    // set at full depth gives while its source sits at rest, so the two must not share a default:
    // EGBIAS[255] is 255, and a positive sense of 3 then buries the operator 35 dB down.
    int out = -1;
    for (int s = 0; s < 8; s++) {
        if (!ctrl_set_active(part, s, 35)) continue;
        int d = perf.c[0x48 + s] - 64;
        int r = (std::abs(scale_f9c(ctrl_set_sum(part, s), d)) + DEPTHTERM[std::min(31, std::abs(d))]) * 2;
        if (r > 0xFD) r = 0xFF;
        out = (~r) & 0xFF;                  // ctrlDest_35 stores DAT_01029738[part]: the last set wins
    }
    return out < 0 ? 0 : out;
}

int Synth::bend_word(const Part& pt) const {
    int b = pt.bend; int r = (b < 0 ? pt.p[0x27] : pt.p[0x26]) - 0x40; int v;
    if (r < 0) { v = BENDTAB[std::min(48, -r)]; if (b >= 0) v = -v; } else { v = BENDTAB[std::min(48, r)]; if (b < 0) v = -v; }
    int k = b >> 3; if (k == 0x7F) k = 0x80;
    int w = (v * k) >> 7;
    w += ctrl_pitch_bias((int)(&pt - perf.part));
    return w;
}

void Synth::part_levels(int part, int& vAtt, int& uAtt) const {
    const Part& pt = perf.part[part];
    int expr = clampi(pt.expr, 0, 254);
    int vol = ctrl_part(part, 17, pt.p[0x0B]);
    int u1 = ((vol + 1) * expr) >> 8; int iu = u1 + 1, iv = u1 + 1;
    int bal = ctrl_part(part, 31, pt.p[0x0A]);
    if (bal < 0x40) iu = ((u1 + 2) * bal) >> 6; else if (bal > 0x40) iv = ((u1 + 2) * ((bal ^ 0x7F) + 1)) >> 6;
    vAtt = VNBAL[clampi(iv, 0, 127)]; uAtt = VNBAL[clampi(iu, 0, 127)];
}

void Synth::refresh_bias(Chan& C, const Part& pt) {
    const Voice& V = pt.voice; int part = C.part;
    int cf = ctrl_offset(part, 36), cv = ctrl_offset(part, 37), cu = ctrl_offset(part, 38), cb = ctrl_eg_bias(part);
    for (int o = 0; o < 8; o++) {
        const OpV& v = V.v[o]; const OpU& u = V.u[o];
        int s = v.fbias - 7;
        C.fbW[o] = (s && (v.form == 7 || v.fixed)) ? (int16_t)(cf * s * 0x10) >> 2 : 0;
        s = u.fbias - 7;
        C.ufbW[o] = (s && u.mode == 0) ? (int16_t)(cf * s * 0x10) >> 2 : 0;
        int bo = BWBIAS[v.bwbias & 0xF] ? clampi((cv * BWBIAS[v.bwbias & 0xF]) >> 3, -128, 127) : 0;
        C.bwReg[o] = clampi(v.bw + bo, 0, 99);
        int ubo = BWBIAS[u.bwbias & 0xF] ? clampi((cu * BWBIAS[u.bwbias & 0xF]) >> 3, -128, 127) : 0;
        C.ubwReg[o] = clampi(eb70(u.bw) + ubo, 0, 127);
        // EG bias attenuation (event 0x205): |sens| * EGBIAS[ctrl or 255-ctrl] >> 3
        auto bias = [&](int sn) { if (!sn) return 0; int idx = sn < 0 ? 255 - cb : cb; return std::min(255, (std::abs(sn) * EGBIAS[idx]) >> 3); };
        C.egbias[o] = bias(v.egbias); C.uegbias[o] = bias(u.egbias);
    }
}
