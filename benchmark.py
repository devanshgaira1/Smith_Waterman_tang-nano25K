#!/usr/bin/env python3
"""
benchmark.py  -  Smith-Waterman FPGA (Tang 25K) vs CPU
                 Supports sequences up to ~1000 bp
                 Baud: 921600, Packet: 0xAA | lenT(2B) | lenD(2B) | target | db
"""
import sys, time, argparse, serial, serial.tools.list_ports

# ── Constants (must match sw_core.v) ─────────────────────────
ENCODE   = {'A':0, 'C':1, 'G':2, 'T':3}
MATCH    =  2
MISMATCH = -1
GAP_OPEN = -4   # total first-gap penalty (matches FPGA GAP_OPEN directly)
GAP_EXT  = -1   # per-base extension penalty

# ── Load sequences ────────────────────────────────────────────
def load_sequences(path):
    pairs, cur_t, cur_d = [], None, None
    with open(path, encoding='utf-8') as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith('#'): continue
            parts = line.split(maxsplit=1)
            if len(parts) < 2: continue
            kw  = parts[0].upper()
            seq = ''.join(c for c in parts[1].upper() if c in 'ACGT')
            if not seq: continue
            if len(seq) > 1000:
                print(f"  WARNING: truncating sequence to 1000 bases")
                seq = seq[:1000]
            if kw == 'TARGET':   cur_t = seq
            elif kw == 'DATABASE': cur_d = seq
            if cur_t and cur_d:
                pairs.append((cur_t, cur_d))
                cur_t = cur_d = None
    return pairs

# ── CPU reference (affine gaps, matches FPGA scoring exactly) ─
# IMPORTANT: E = max(H + GAP_OPEN, E + GAP_EXT)  -- NOT H+GAP_OPEN+GAP_EXT
# Because FPGA new_P = max(D + GAP_OPEN, P + GAP_EXT) where GAP_OPEN = -4
def sw_cpu(seqA, seqB):
    A = '-' + seqA;  B = '-' + seqB
    m, n = len(A), len(B)
    NI = -(1 << 30)
    H = [[0]*n for _ in range(m)]
    E = [[NI]*n for _ in range(m)]
    F = [[NI]*n for _ in range(m)]
    best = 0
    for i in range(1, m):
        for j in range(1, n):
            s       = MATCH if A[i]==B[j] else MISMATCH
            E[i][j] = max(H[i][j-1] + GAP_OPEN,   E[i][j-1] + GAP_EXT)
            F[i][j] = max(H[i-1][j] + GAP_OPEN,   F[i-1][j] + GAP_EXT)
            H[i][j] = max(0, H[i-1][j-1]+s, E[i][j], F[i][j])
            if H[i][j] > best: best = H[i][j]
    return best

def run_cpu(pairs):
    scores = []
    t0 = time.perf_counter()
    for a, b in pairs:
        scores.append(sw_cpu(a, b))
    return scores, time.perf_counter() - t0

# ── Encode sequence (4-bit, in-order, no sentinel) ────────────
def encode_seq(seq):
    return bytes(ENCODE.get(c, 0) for c in seq)

# ── Auto-detect CH340/CH341 port ─────────────────────────────
def find_fpga_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or '').upper()
        if 'CH340' in desc or 'CH341' in desc:
            return p.device
    return None

# ── FPGA communication ────────────────────────────────────────
def run_fpga(pairs, port, baud=921600):
    scores = []
    t0 = time.perf_counter()
    with serial.Serial(port, baud, timeout=10) as ser:
        print(f"  Connected to {port} @ {baud} baud")
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        for i, (target, db) in enumerate(pairs, 1):
            lenT = len(target)
            lenD = len(db)
            packet = (bytes([0xAA])
                    + lenT.to_bytes(2, 'big')
                    + lenD.to_bytes(2, 'big')
                    + encode_seq(target)
                    + encode_seq(db))
            print(f"  Pair {i:>2} ({lenT}×{lenD}) ... ", end='', flush=True)
            ser.reset_input_buffer()
            ser.write(packet)
            ser.flush()
            resp = ser.read(3)
            if len(resp) != 3 or resp[0] != 0xBB:
                got = f"0x{resp[0]:02X}" if resp else "nothing"
                print(f"ERROR (got {got})")
                scores.append(-9999)
                continue
            score = (resp[1] << 8) | resp[2]
            print(f"score = {score}")
            scores.append(score)
    return scores, time.perf_counter() - t0

# ── GCUPS ─────────────────────────────────────────────────────
def gcups(pairs, elapsed):
    if elapsed <= 0: return 0.0
    return sum((len(a)+1)*(len(b)+1) for a,b in pairs) / elapsed / 1e9

# ── Results table ─────────────────────────────────────────────
def print_results(pairs, cpu_scores, cpu_time, fpga_scores=None, fpga_time=None):
    n = len(pairs)
    print("\n" + "="*72)
    print("  SMITH-WATERMAN  —  Tang 25K FPGA  vs  CPU (Python)")
    print("="*72)
    print(f"  Pairs   : {n}   Scoring: match={MATCH} mismatch={MISMATCH} "
          f"gap_open={GAP_OPEN} gap_ext={GAP_EXT}")
    print()
    print(f"  {'Platform':<22} {'Time (s)':>10}   {'GCUPS':>12}")
    print("  " + "-"*22 + " " + "-"*10 + "   " + "-"*12)
    print(f"  {'CPU (Python)':<22} {cpu_time:>10.3f}   {gcups(pairs,cpu_time):>12.6f}")
    if fpga_scores and fpga_time:
        n_wrong = sum(1 for c,f in zip(cpu_scores,fpga_scores) if f>=0 and c!=f)
        n_fail  = sum(1 for f in fpga_scores if f<0)
        print(f"  {'FPGA (Tang 25K)':<22} {fpga_time:>10.3f}   {gcups(pairs,fpga_time):>12.6f}")
        print()
        print(f"  Speedup        : {cpu_time/fpga_time:.1f}×")
        print(f"  Correct scores : {n-n_wrong-n_fail}/{n}")
    print()
    print(f"  {'#':>3}  {'Target':>12}  {'Database':>12}  {'CPU':>6}", end="")
    if fpga_scores: print(f"  {'FPGA':>6}  {'OK?':>4}", end="")
    print()
    print("  " + "-"*55)
    for i,(a,b) in enumerate(pairs):
        ta = (a[:10]+'..') if len(a)>12 else a
        tb = (b[:10]+'..') if len(b)>12 else b
        row = f"  {i+1:>3}  {ta:>12}  {tb:>12}  {cpu_scores[i]:>6}"
        if fpga_scores:
            f  = fpga_scores[i]
            ok = '✓' if f==cpu_scores[i] else ('?' if f<0 else '✗')
            row += f"  {f:>6}  {ok:>4}"
        print(row)
    print("="*72)
    with open("benchmark_results.csv","w") as f:
        f.write("pair,lenA,lenB,cpu_score" + (",fpga_score,correct" if fpga_scores else "") + "\n")
        for i,(a,b) in enumerate(pairs):
            row = f"{i+1},{len(a)},{len(b)},{cpu_scores[i]}"
            if fpga_scores:
                row += f",{fpga_scores[i]},{1 if fpga_scores[i]==cpu_scores[i] else 0}"
            f.write(row+"\n")
    print(f"\n  Saved: benchmark_results.csv\n")

# ── Main ──────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port",     default=None)
    p.add_argument("--baud",     type=int, default=921600)
    p.add_argument("--seqfile",  default="sequences.txt")
    p.add_argument("--cpu-only", action="store_true")
    args = p.parse_args()

    print(f"\nLoading '{args.seqfile}' ...")
    pairs = load_sequences(args.seqfile)
    if not pairs: print("No valid pairs."); return
    print(f"  {len(pairs)} pairs loaded.\n")

    print("Running CPU reference ...")
    cpu_scores, cpu_time = run_cpu(pairs)
    print(f"  Done: {cpu_time:.3f} s")
    for i,(s,(a,b)) in enumerate(zip(cpu_scores,pairs)):
        print(f"    Pair {i+1:>2}: CPU = {s:>5}  ({len(a)}×{len(b)} bp)")

    if args.cpu_only:
        print_results(pairs, cpu_scores, cpu_time); return

    port = args.port or find_fpga_port()
    if not port:
        ports = list(serial.tools.list_ports.comports())
        print("\nAvailable ports:")
        for i,pp in enumerate(ports): print(f"  [{i}] {pp.device}  {pp.description}")
        port = input("Enter port (e.g. COM8): ").strip()

    print(f"\n{'━'*55}")
    print("  FPGA STEPS:")
    print("  1. Open Gowin Programmer")
    print("  2. Load bitstream → Program → wait for 'Success'")
    print("  3. Type  go  and press Enter")
    print(f"{'━'*55}")
    while True:
        cmd = input("\n  > ").strip().lower()
        if cmd == 'go': break
        if cmd in ('q','quit'): print_results(pairs,cpu_scores,cpu_time); return
        print("  Type  go  to run FPGA, or  quit  to show CPU only.")

    print(f"\nRunning FPGA on {port} @ {args.baud} baud ...")
    try:
        fpga_scores, fpga_time = run_fpga(pairs, port, args.baud)
    except Exception as e:
        print(f"  Error: {e}")
        fpga_scores = fpga_time = None

    print_results(pairs, cpu_scores, cpu_time, fpga_scores, fpga_time)

if __name__ == "__main__":
    main()