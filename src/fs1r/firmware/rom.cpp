// fs1r/firmware/rom.cpp - EPROM and sysex loading, and the parameter map. KNOWN.
#include "fs1r/internal.h"

bool load_rom(Rom& R, const char* path) {
    FILE* f = fopen(path, "rb"); if (!f) return false;
    R.d.assign(2097152, 0); size_t n = fread(R.d.data(), 1, R.d.size(), f); fclose(f);
    if (n != R.d.size()) return false;
    if (R.d[0x1C8000] == 0 && R.d[0x1C8001] == 0 && R.d[0x1C8002] == 0 && R.d[0x1C8003] == 4)
        for (size_t i = 0; i < R.d.size(); i += 2) std::swap(R.d[i], R.d[i + 1]);
    R.ok = true; return true;
}

void rom_voice(const Rom& R, int idx, Voice& V) {
    idx = clampi(idx, 0, 1407);
    if (idx < 256) memcpy(V.raw, R.d.data() + 0x5C280 + (size_t)idx * 608, 608);
    else convert_dx7(R.d.data() + 0x30000 + (size_t)(idx - 256) * 155, V.raw);   // 155-byte VCED, name last, as tools/extract_presets.py reads it
    decode_voice(V);
}

int bank_voice_index(int bank, int prog) {
    prog &= 0x7F;
    if (bank == 2 || bank == 3) return (bank - 2) * 128 + prog;        // PrA, PrB: native
    if (bank >= 4 && bank <= 12) return 256 + (bank - 4) * 128 + prog; // PrC..PrK: DX7
    return prog;
}

void perf_from_bytes(Synth& S, const Rom* R, const uint8_t* d) {
    Perf& P = S.perf;
    memcpy(P.c, d, 80); memcpy(P.fx, d + 80, 112);
    for (int i = 0; i < 4; i++) {
        Part& pt = P.part[i]; memcpy(pt.p, d + 192 + 52 * i, 52); pt.nheld = 0; pt.lastPitch = -1; pt.reset_ctl();
        if (R && R->ok && pt.p[1]) rom_voice(*R, bank_voice_index(pt.p[1], pt.p[2]), pt.voice);
    }
    memcpy(P.name, P.c, 12); P.name[12] = 0;
    S.fseqPart = (P.c[0x15] & 7) ? (P.c[0x15] & 7) - 1 : -1;
    if (R && R->ok && S.fseqPart >= 0 && (P.c[0x16] & 1)) {
        int n = P.c[0x17]; size_t off = n < 10 ? 0x100A00 + (size_t)n * 25632 : 0x83000 + (size_t)(n - 10) * 6432;
        S.fseq.from_bytes(R->d.data() + off, R->d.data() + off + 32, n < 10 ? 512 : 128);
    }
}

bool rom_perf(Synth& S, const Rom& R, int idx) {
    idx = clampi(idx, 0, 383); size_t off = 0xA000 + (size_t)idx * 400;
    std::lock_guard<std::mutex> lk(S.mtx);
    perf_from_bytes(S, &R, R.d.data() + off);
    return true;
}

bool rom_ready(const Rom* R) { return R && R->ok; }

void Synth::load_perf_bank(int lsb, int prog) {
    if (!rom_ready(rom)) return;
    perf_from_bytes(*this, rom, rom->d.data() + 0xA000 + (size_t)bank_perf_index(lsb, prog) * 400);
}

bool rom_fseq(Synth& S, const Rom& R, int n) {
    n = clampi(n, 1, 90); size_t off = n <= 10 ? 0x100A00 + (size_t)(n - 1) * 25632 : 0x83000 + (size_t)(n - 11) * 6432;
    std::lock_guard<std::mutex> lk(S.mtx); S.fseq.from_bytes(R.d.data() + off, R.d.data() + off + 32, n <= 10 ? 512 : 128);
    if (S.fseqPart < 0) S.fseqPart = 0;
    return true;
}

bool load_sysex(Synth& S, const Rom* R, const uint8_t* d, size_t len, int pick, int part) {
    // FS1R bulk: F0 43 0n 5E bc bc ah am al <data> cs F7. Voice 608 bytes (ah 40-43/51), performance 400 (ah 10/11), Fseq
    // (ah 60 current, 61 internal: the Data List's "6b 00 nn", 3.2.1; and 70, which tools/extract_presets.py wrote the
    // bundled presets with before the Data List was checked, kept so they still load)
    // DX7 VCED:  F0 43 0n 00 01 1B <155 bytes> cs F7
    int found = 0;
    for (size_t i = 0; i + 10 < len; i++) {
        if (d[i] != 0xF0 || d[i + 1] != 0x43 || (d[i + 2] & 0xF0) != 0x00) continue;
        if (d[i + 3] == 0x5E) {
            int bc = d[i + 4] << 7 | d[i + 5]; int ah = d[i + 6]; const uint8_t* p = d + i + 9;
            if (i + 9 + bc + 2 > len) break;
            if (bc == 608 && ((ah >= 0x40 && ah <= 0x43) || ah == 0x51)) {
                if (found++ != pick) continue;
                int pi = ah <= 0x43 ? ah - 0x40 : part;
                memcpy(S.perf.part[pi].voice.raw, p, 608); decode_voice(S.perf.part[pi].voice); return true;
            }
            if (bc == 400 && (ah == 0x10 || ah == 0x11)) {
                if (found++ != pick) continue;
                perf_from_bytes(S, R, p); return true;
            }
            // "FSeq Bulk does not interpret Byte Count" (Data List 3.2.1), and it cannot: a 512 frame
            // dump is 25632 bytes and the count is 14 bits. The header's own frame count says how long
            // the dump is, which is also how the length is known when stepping over one.
            if ((ah == 0x60 || ah == 0x61 || ah == 0x70) && i + 9 + 32 <= len) {
                int frames = 128 * ((p[0x1B] & 3) + 1);
                size_t n = 32 + (size_t)frames * 50;
                if (i + 9 + n + 2 > len) break;
                if (found++ != pick) { i += 10 + n; continue; }
                S.fseq.from_bytes(p, p + 32, frames); if (S.fseqPart < 0) S.fseqPart = 0; return true;
            }
        } else if (d[i + 3] == 0x05 && d[i + 4] == 0x00 && d[i + 5] == 0x31) {
            if (i + 6 + 49 + 2 > len) break;
            memcpy(g_aced, d + i + 6, 49); g_acedValid = true;       // held for the next VCED
            i += 6 + 49 + 1;
        } else if (d[i + 3] == 0x00 && d[i + 4] == 0x01 && d[i + 5] == 0x1B) {
            if (i + 6 + 155 + 2 > len) break;
            if (found++ != pick) continue;
            convert_dx7(d + i + 6, S.perf.part[part].voice.raw); decode_voice(S.perf.part[part].voice); return true;
        }
    }
    return false;
}

bool param_is_wide(int ah, int am, int al) {
    if (ah == 0x10 && am == 0) return al == 0x18 || al == 0x1A || al == 0x1C || al == 0x1E ||
                                      (al >= 0x30 && al <= 0x3E && !(al & 1));
    if (ah == 0x10 && am == 0 && al >= 0x50) return !((al - 0x50) & 1) && al <= 0x5F;
    if (ah == 0x10 && am == 1) return al <= 0x27 && !(al & 1);
    if (ah == 0x70 && am == 0) return al == 0x10 || al == 0x12 || al == 0x1E;
    return false;
}

bool write_param(Synth& S, int ah, int am, int al, int val) {
    uint8_t v = (uint8_t)(val & 0x7F);
    if (ah == 0x00 && am == 0 && al < 76) { if (al != 0x46) S.sys[al] = v; return true; }
    if (ah == 0x10 && am == 0 && al < 80) {
        S.perf.c[al] = v;
        if (al == 0x15 || al == 0x16 || al == 0x17) {
            S.fseqPart = (S.perf.c[0x15] & 7) ? (S.perf.c[0x15] & 7) - 1 : -1;
            if (rom_ready(S.rom) && (S.perf.c[0x16] & 1)) {
                int n = S.perf.c[0x17];
                size_t off = n < 10 ? 0x100A00 + (size_t)n * 25632 : 0x83000 + (size_t)(n - 10) * 6432;
                S.fseq.from_bytes(S.rom->d.data() + off, S.rom->d.data() + off + 32, n < 10 ? 512 : 128);
            }
        }
        return true;
    }
    if (ah == 0x10 && ((am == 0 && al >= 0x50) || am == 1)) { int i = am ? 48 + al : al - 0x50; if (i >= 112) return false; S.perf.fx[i] = v; return true; }
    if (ah >= 0x30 && ah <= 0x33 && am == 0 && al < 52) { S.perf.part[ah - 0x30].p[al] = v; return true; }
    if (ah >= 0x40 && ah <= 0x43 && am == 0 && al < 112) { Voice& V = S.perf.part[ah - 0x40].voice; V.raw[al] = v; decode_voice(V); S.voice_changed(ah - 0x40); return true; }
    if (ah >= 0x60 && ah <= 0x63 && am < 8 && al < 62) { Voice& V = S.perf.part[ah - 0x60].voice; V.raw[112 + am * 62 + al] = v; decode_voice(V); S.voice_changed(ah - 0x60); return true; }
    if (ah == 0x70 && am == 0 && al < 32) {                       // Fseq header
        uint8_t h[32] = {}; std::vector<uint8_t> cur; S.fseq_bytes(cur);
        if (cur.size() >= 32) memcpy(h, cur.data(), 32);
        h[al] = v;
        if (S.fseq.valid) S.fseq.from_bytes(h, (const uint8_t*)S.fseq.frame, S.fseq.nframes);
        return true;
    }
    return false;
}

bool apply_param_change_locked(Synth& S, const uint8_t* d, size_t len) {
    if (len < 8 || d[0] != 0xF0 || d[1] != 0x43 || d[3] != 0x5E) return false;
    int kind = d[2] & 0xF0, dev = d[2] & 0x0F;
    if (S.devNumber() != 0x10 && S.devNumber() != dev) return false;
    int ah = d[4], am = d[5], al = d[6];
    if (kind == 0x30) { int v = S.read_param(ah, am, al); if (v >= 0) S.push_param(ah, am, al, v); return v >= 0; }
    if (kind == 0x20) { S.dump_request(ah, am, al); return true; }
    if (kind != 0x10 || len < 10) return false;
    int val = d[7] << 7 | d[8];
    if (param_is_wide(ah, am, al)) return write_param(S, ah, am, al, val >> 7) & write_param(S, ah, am, al + 1, val);
    return write_param(S, ah, am, al, val);
}

bool apply_param_change(Synth& S, const uint8_t* d, size_t len) {
    std::lock_guard<std::mutex> lk(S.mtx);
    return apply_param_change_locked(S, d, len);
}

