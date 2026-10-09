/* SPDX-License-Identifier: GPL-3.0-only */
/* The fork: DX7 voices measured as tools/level_presets.py measures the factory presets (tests/preset_level.c's
 * phrases and roles, through the whole mix), as a voice plays after a PRESETS pick: FM6 preset 0's settings, the
 * voice as the patch, dry (the sends 0), P_ED_FX 0. Also prints the firmware's own estimate (eng_fm6.c dx_est) and hash (dx_hash).
 *   cc -O2 -Itests -Ibuild/gen -Ifirmware/src -Ifirmware/hal tests/dx_level.c -lm -o build/host/dx_level
 *   TRIMMED=1 build/host/dx_level OUT.raw OUT.txt BANK.syx...   (then tools/level_dx.py OUT.raw OUT.txt)
 * TRIMMED: each voice plays with the trim the firmware gives it (dx_trim_of), so the tool adds what is left */
#define main hostsim_main
#include "hostsim.c"
#undef main

enum { R_BASS, R_HELD, R_COMP, R_MELODY, R_ONE };
static int has(const char *n, const char *w) { return strstr(n, w) != 0; }
static int role_of(const char *n)
{
    if (has(n, "BASS") || has(n, "808") || has(n, "ACID") || has(n, "REESE") || has(n, "WOBBLE") || has(n, "SUB") || has(n, "BOOM"))
        return R_BASS;
    if (has(n, "PAD") || has(n, "STR") || has(n, "CHOIR") || has(n, "OOH") || has(n, "ORGAN") || has(n, "ORGN") || has(n, "GOSPEL") ||
        has(n, "B3") || has(n, "CLOUD") || has(n, "HAZE") || has(n, "ATMOS") || has(n, "ENSEMBLE") || has(n, "DRONE") ||
        has(n, "SHIMMER") || has(n, "OCEAN") || has(n, "RISER") || has(n, "WIND") || has(n, "HISS") || has(n, "VINYL") ||
        has(n, "RAIN") || has(n, "BOWL") || has(n, "BOW") || has(n, "TANPURA"))
        return R_HELD;
    if ((has(n, "STAB") && !has(n, "HORN") && !has(n, "STRING")) || has(n, "CHORD"))
        return R_ONE;                                  /* TRIO stabs: one key plays the chord */
    if (has(n, "RHODES") || has(n, "WURLI") || has(n, "CLAV") || has(n, "KEYS") || has(n, "STAB") || has(n, "PNO") || has(n, "PIANO") ||
        has(n, "VIBES") || has(n, "BRASS") || has(n, "HORN") || has(n, "ACCORDION") || has(n, "POLY"))
        return R_COMP;
    return R_MELODY;
}

typedef struct { uint8_t step, note, len; } ev_t;   /* 1/16 steps at 100 BPM */
static const ev_t BASS[] = {{0, 36, 3}, {4, 36, 1}, {6, 43, 2}, {8, 41, 3}, {12, 39, 2}, {14, 38, 2},
                            {16, 36, 3}, {20, 36, 1}, {22, 43, 2}, {24, 46, 3}, {28, 43, 4}};
static const ev_t MELODY[] = {{0, 72, 2}, {2, 75, 2}, {4, 77, 3}, {8, 79, 2}, {10, 77, 2}, {12, 75, 4},
                              {16, 72, 2}, {18, 70, 2}, {20, 72, 3}, {24, 67, 4}, {28, 70, 2}, {30, 72, 2}};
static const uint8_t CHORD[2][4] = {{60, 63, 67, 70}, {58, 62, 65, 68}};   /* Cm7, Bb7sus-ish */
static const uint8_t COMP[] = {0, 3, 6, 10, 12, 16, 19, 22, 26, 28};

static uint8_t held[128];
static uint32_t off_at[128];
static void play(uint32_t note, uint32_t vel, uint32_t off)
{
    trk_note_on(&trk[0], note, vel);
    held[note] = 1;
    off_at[note] = off;
}


int main(int argc, char **argv)
{
    /* argv: out.raw out.txt bank.syx... : each voice through the whole mix on the phrase of its role (ROLE=n forces
     * one), as a DX voice plays after a PRESETS pick: FM6 preset 0's settings, the voice as the patch, P_ED_FX = 0 */
    FILE *rep = fopen(argv[2], "w"), *raw = fopen(argv[1], "wb");
    const uint32_t step = FS * 60u / 100u / 4u, nb = (FS * 9u / 2u) / CTL, tail = (FS / 4u) / CTL;
    int32_t o[2 * CTL];
    for (int f = 3; f < argc; f++) {
        FILE *fp = fopen(argv[f], "rb"); uint8_t syx[4104]; if (!fp || fread(syx, 1, 4104, fp) != 4104) return 2; fclose(fp);
        for (uint32_t k = 0; k < 32u; k++) {
            uint8_t v[FP_SIZE + 1]; char name[11]; uint32_t b, n, i;
            const uint8_t *pk = syx + 6 + k * 128;
            memcpy(name, pk + 118, 10); name[10] = 0;
            int role = getenv("ROLE") ? atoi(getenv("ROLE")) : role_of(name);
            host_tracks_init();
            song.g[G_BPM] = 100;
            host_preset(&trk[0], ENGI_FM6, 0);
            for (i = 0; i < 4u; i++)
                trk[0].p[P_DIST + i] = 0;                 /* dry, as a PRESETS pick leaves a DX7 voice (ui.c preset_go) */
            trk[0].p[P_ED_FX] = getenv("TRIMMED") ? (int16_t)dx_trim_of(pk) : 0;   /* TRIMMED: as the firmware sets it */
            fm6_unpack(pk, v);
            fm6_set_patch(0, v);
            song.sel = 0;
            memset(held, 0, sizeof held);
            for (b = 0; b < FS / CTL; b++) mix_block(o, CTL);
            fprintf(rep, "%s %u %d %08x %d %d %s\n", argv[f], k + 1, role, (unsigned)dx_hash(pk), (int)dx_est(pk), (int)trk[0].p[P_ED_FX], name);
            for (b = 0; b < nb + tail; b++) {
                uint32_t s0 = b * CTL, s1 = s0 + CTL, st;
                for (n = 0; n < 128u; n++)
                    if (held[n] && (off_at[n] < s1 || b >= nb)) { trk_note_off(&trk[0], n); held[n] = 0; }
                for (st = 0; st < 32u && b < nb; st++) {
                    uint32_t at = st * step;
                    if (at < s0 || at >= s1) continue;
                    if (role == R_BASS || role == R_MELODY) {
                        const ev_t *ev = role == R_BASS ? BASS : MELODY;
                        uint32_t ne = role == R_BASS ? sizeof BASS / sizeof BASS[0] : sizeof MELODY / sizeof MELODY[0];
                        for (i = 0; i < ne; i++) if (ev[i].step == st) play(ev[i].note, 100, at + ev[i].len * step - step / 4u);
                    } else if (role == R_HELD) {
                        if (st == 0 || st == 16) for (i = 0; i < 4u; i++) play(CHORD[st / 16u][i], 90, at + 15u * step);
                    } else {
                        for (n = 0; n < sizeof COMP; n++) if (COMP[n] == st)
                            for (i = 0; i < (role == R_ONE ? 1u : 4u); i++) play(CHORD[st / 16u][i], 95, at + 2u * step - step / 3u);
                    }
                }
                mix_block(o, CTL);
                for (i = 0; i < CTL; i++) { int32_t m = (o[2 * i] + o[2 * i + 1]) / 2; fwrite(&m, 4, 1, raw); }
            }
        }
    }
    fclose(rep); fclose(raw);
    return 0;
}
