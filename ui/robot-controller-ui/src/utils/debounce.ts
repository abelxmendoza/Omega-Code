/*
# File: /src/utils/debounce.ts
# Summary:
#   Small, dependency-free debounce with TypeScript types.
#   - debounce(fn, wait, { leading?, trailing?, maxWait? })
#   - Returns a debounced function with .cancel(), .flush(), .pending()
#   - React hook: useDebouncedCallback(fn, wait, opts, deps?)
#
# Why this version?
#   • Uses a monotonic clock (performance.now) to avoid wall-clock jumps
#   • Correct leading-only behavior (no accidental trailing call)
#   • Solid maxWait handling (guarantees at least one invoke per window)
#   • No JSON.stringify in deps; stable, typed, SSR-safe
*/

export type DebounceOptions = {
  /** Invoke on the leading edge (immediately). Default: false */
  leading?: boolean;
  /** Invoke on the trailing edge (after silence). Default: true */
  trailing?: boolean;
  /** Guarantee at least one invoke within this window (ms). */
  maxWait?: number;
};

export type Debounced<T extends (...args: any[]) => any> = {
  (...args: Parameters<T>): void;
  /** Cancel any pending invocation and reset internal state. */
  cancel(): void;
  /** If pending, immediately invoke with the latest args. */
  flush(): void;
  /** True while a timer is active. */
  pending(): boolean;
};

const now = () =>
  typeof performance !== 'undefined' && performance.now ? performance.now() : Date.now();

/**
 * Core debounce
 */
export function debounce<T extends (...args: any[]) => any>(
  fn: T,
  wait: number,
  opts: DebounceOptions = {}
): Debounced<T> {
  const leading = !!opts.leading;
  const trailing = opts.trailing !== false; // default true
  const maxWait = typeof opts.maxWait === 'number' && opts.maxWait > 0 ? opts.maxWait : undefined;
  const delay = Math.max(0, Number.isFinite(wait) ? wait : 0);

  let timer: ReturnType<typeof setTimeout> | null = null;
  let lastArgs: Parameters<T> | null = null;
  let lastThis: any = null;

  // Monotonic timestamps
  let lastInvokeTs = -1;  // last time fn actually ran
  let startTs = -1;       // time of first call in current window

  function invoke(ts: number) {
    lastInvokeTs = ts;
    const args = lastArgs!;
    const ctx = lastThis;
    lastArgs = lastThis = null;
    try {
      // We intentionally don't capture/return the result, the type is void-like for callers.
      (fn as any).apply(ctx, args);
    } catch {
      // Intentionally swallow: users handle their own try/catch inside fn if needed.
    }
  }

  function clearTimer() {
    if (timer) clearTimeout(timer);
    timer = null;
  }

  function remainingWait(ts: number) {
    const sinceInvoke = ts - lastInvokeTs;
    const sinceStart = ts - startTs;
    const waitRemaining = delay - sinceInvoke;
    return maxWait !== undefined ? Math.min(waitRemaining, maxWait - sinceStart) : waitRemaining;
  }

  function shouldInvoke(ts: number) {
    if (lastInvokeTs < 0) return true; // never invoked
    const sinceInvoke = ts - lastInvokeTs;
    return sinceInvoke >= delay || sinceInvoke < 0;
  }

  function onTimeout() {
    const ts = now();

    // If we still have args waiting and we've satisfied the remaining wait,
    // execute the trailing edge; otherwise reschedule.
    if (lastArgs && trailing && remainingWait(ts) <= 0) {
      trailingEdge(ts);
      return;
    }

    if (lastArgs) {
      // Still receiving calls; reschedule for the remaining wait
      const ms = Math.max(0, remainingWait(ts));
      clearTimer();
      timer = setTimeout(onTimeout, ms);
    } else {
      // Nothing pending; clear state timer
      clearTimer();
    }
  }

  function leadingEdge(ts: number) {
    startTs = ts;
    // Schedule a trailing call if needed
    clearTimer();
    timer = setTimeout(onTimeout, delay);

    if (leading) {
      // Leading edge call happens immediately
      invoke(ts);
    }
  }

  function trailingEdge(ts: number) {
    clearTimer();
    // Only invoke if we still have pending args and trailing is enabled
    if (trailing && lastArgs) {
      invoke(ts);
    }
  }

  const debounced = function (this: any, ...args: Parameters<T>) {
    const ts = now();
    lastArgs = args;
    lastThis = this;

    if (startTs < 0) startTs = ts; // seed start on the very first call of a burst

    const invokeNow = shouldInvoke(ts);

    if (!timer) {
      // No timer active → we’re starting a new burst (or coming back after flush/cancel)
      leadingEdge(ts);
    }

    // If maxWait is set, make sure we don't exceed it without running
    if (maxWait !== undefined && ts - startTs >= maxWait) {
      trailingEdge(ts);
      leadingEdge(ts);
      return;
    }

    // If we should invoke immediately (based on classic wait), do so (for leading-only scenarios)
    if (invokeNow && leading && !trailing) {
      // For leading-only, we already invoked on leadingEdge; just restart window
      startTs = ts;
      clearTimer();
      timer = setTimeout(onTimeout, delay);
    }
  } as Debounced<T>;

  debounced.cancel = () => {
    clearTimer();
    lastArgs = lastThis = null;
    startTs = lastInvokeTs = -1;
  };

  debounced.flush = () => {
    if (!timer) return;
    const ts = now();
    trailingEdge(ts);
  };

  debounced.pending = () => !!timer;

  return debounced;
}

/* -------------------------------- React hook -------------------------------- */

import { useEffect, useMemo, useRef } from 'react';

/**
 * useDebouncedCallback:
 * - Stable debounced function across renders.
 * - Re-creates when wait/options or explicit deps change.
 * - Cancels timers on unmount to avoid late calls.
 *
 * Example:
 *   const onTune = useDebouncedCallback(setParams, 250, { trailing: true }, [mode]);
 */
export function useDebouncedCallback<T extends (...args: any[]) => any>(
  fn: T,
  wait: number,
  opts?: DebounceOptions,
  deps: any[] = []
): Debounced<T> {
  const fnRef = useRef(fn);
  fnRef.current = fn;

  // Normalize options so deps are stable without JSON.stringify
  const leading = !!opts?.leading;
  const trailing = opts?.trailing !== false;
  const maxWait = typeof opts?.maxWait === 'number' && opts?.maxWait > 0 ? opts?.maxWait : undefined;

  const debounced = useMemo(
    () => debounce(((...a: Parameters<T>) => fnRef.current(...a)) as T, wait, { leading, trailing, maxWait }),
    // Rebuild only when these primitives (and user deps) change
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [wait, leading, trailing, maxWait, ...deps]
  );

  useEffect(() => () => debounced.cancel(), [debounced]);
  return debounced;
}

/* ------------------------------ Bonus helper ------------------------------ */
/**
 * Debounce a changing value and get the debounced copy back.
 * Useful for search inputs or sliders when you don't need manual .flush().
 */
export function useDebouncedValue<T>(value: T, wait = 250): T {
  const [v, setV] = (React as any).useState<T>(value);
  const setDebounced = useDebouncedCallback(setV, wait, { trailing: true }, []);
  useEffect(() => { setDebounced(value); }, [value, setDebounced]);
  return v;
}
