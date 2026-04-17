/**
 * /api/sim-launcher — manage the local simulation backend process.
 *
 * GET    → { alive, simMode, pid, ownedProcess }
 * POST   → start backend with SIM_MODE=1 (returns { started, pid })
 * DELETE → stop the locally-started backend (returns { stopped, pid })
 *
 * "Owned" means it was started by this API route (PID tracked in PID_FILE).
 * Backends started externally (e.g. from the Pi or a terminal) are reported
 * as alive but not owned — the Stop button is hidden in that case.
 */

import type { NextApiRequest, NextApiResponse } from 'next';
import { spawn }                                from 'child_process';
import path                                     from 'path';
import fs                                       from 'fs';

// ---------------------------------------------------------------------------
// Paths
// ---------------------------------------------------------------------------

const PID_FILE   = '/tmp/omega-sim-backend.pid';

// cwd() inside Next.js API routes = the Next.js project root
// ui/robot-controller-ui → ../../servers/robot_controller_backend
const BACKEND_DIR  = path.resolve(process.cwd(), '../../servers/robot_controller_backend');
const START_SCRIPT = path.join(BACKEND_DIR, 'scripts', 'start_sim_local.sh');

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

async function checkBackend(): Promise<{ alive: boolean; simMode: boolean }> {
  try {
    const ctrl = new AbortController();
    const id   = setTimeout(() => ctrl.abort(), 2000);
    const res  = await fetch('http://localhost:8000/health', { signal: ctrl.signal });
    clearTimeout(id);
    if (!res.ok) return { alive: false, simMode: false };

    // probe sim routes — 200 means SIM_MODE=1 is active
    const ctrl2 = new AbortController();
    const id2   = setTimeout(() => ctrl2.abort(), 2000);
    const sim   = await fetch('http://localhost:8000/sim/status', { signal: ctrl2.signal });
    clearTimeout(id2);
    return { alive: true, simMode: sim.ok };
  } catch {
    return { alive: false, simMode: false };
  }
}

function readPid(): number | null {
  try {
    const n = parseInt(fs.readFileSync(PID_FILE, 'utf-8').trim(), 10);
    return Number.isFinite(n) ? n : null;
  } catch {
    return null;
  }
}

function pidAlive(pid: number): boolean {
  try { process.kill(pid, 0); return true; } catch { return false; }
}

// ---------------------------------------------------------------------------
// Handler
// ---------------------------------------------------------------------------

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  // ── GET: current status ─────────────────────────────────────────────────
  if (req.method === 'GET') {
    const { alive, simMode } = await checkBackend();
    const pid          = readPid();
    const ownedProcess = pid !== null && pidAlive(pid);
    res.json({ alive, simMode, pid: ownedProcess ? pid : null, ownedProcess });
    return;
  }

  // ── POST: start backend ─────────────────────────────────────────────────
  if (req.method === 'POST') {
    const { alive } = await checkBackend();
    if (alive) {
      res.status(409).json({ error: 'Backend already running on port 8000' });
      return;
    }

    if (!fs.existsSync(START_SCRIPT)) {
      res.status(500).json({
        error: `Start script not found at ${START_SCRIPT}`,
      });
      return;
    }

    // spawn detached so it outlives the Next.js process
    const child = spawn('bash', [START_SCRIPT], {
      detached: true,
      stdio:    'ignore',
      cwd:      BACKEND_DIR,
    });
    child.unref();

    if (!child.pid) {
      res.status(500).json({ error: 'spawn() returned no PID' });
      return;
    }

    fs.writeFileSync(PID_FILE, String(child.pid), 'utf-8');
    res.status(201).json({ started: true, pid: child.pid });
    return;
  }

  // ── DELETE: stop owned backend ──────────────────────────────────────────
  if (req.method === 'DELETE') {
    const pid = readPid();
    if (pid === null || !pidAlive(pid)) {
      try { fs.unlinkSync(PID_FILE); } catch { /* ignore */ }
      res.status(404).json({ error: 'No locally-owned backend process found' });
      return;
    }
    try {
      process.kill(pid, 'SIGTERM');
      fs.unlinkSync(PID_FILE);
      res.json({ stopped: true, pid });
    } catch (e) {
      res.status(500).json({ error: String(e) });
    }
    return;
  }

  res.status(405).end();
}
