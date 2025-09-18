import React, {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
  type ReactNode,
} from 'react';
import { v4 as uuidv4 } from 'uuid';
import { useCommand } from './CommandContext';

const STORAGE_KEY = 'omega.macros.v1';

export interface MacroStep {
  id: string;
  command: string;
  label?: string;
  payload?: Record<string, any>;
  delayMs?: number;
}

export interface Macro {
  id: string;
  name: string;
  description?: string;
  steps: MacroStep[];
  createdAt: number;
  updatedAt: number;
}

type RuntimeState = {
  runningId: string | null;
  stepIndex: number;
  startedAt: number | null;
};

type MacroContextType = {
  macros: Macro[];
  isLoaded: boolean;
  createMacro: (name?: string, description?: string) => Macro;
  updateMacro: (
    id: string,
    updates: Partial<Omit<Macro, 'id' | 'createdAt' | 'updatedAt' | 'steps'>> & {
      steps?: MacroStep[];
    },
  ) => void;
  deleteMacro: (id: string) => void;
  duplicateMacro: (id: string) => Macro | undefined;
  addStep: (macroId: string, step: MacroStep) => void;
  updateStep: (macroId: string, stepId: string, updates: Partial<MacroStep>) => void;
  removeStep: (macroId: string, stepId: string) => void;
  moveStep: (macroId: string, fromIndex: number, toIndex: number) => void;
  runMacro: (id: string) => Promise<void>;
  stopMacro: () => void;
  runtime: RuntimeState;
  isRunning: boolean;
  stopRequested: boolean;
};

const MacroContext = createContext<MacroContextType | undefined>(undefined);

export const useMacro = (): MacroContextType => {
  const ctx = useContext(MacroContext);
  if (!ctx) {
    throw new Error('useMacro must be used within a MacroProvider');
  }
  return ctx;
};

const cloneSteps = (steps: MacroStep[]): MacroStep[] =>
  steps.map((step) => ({ ...step, id: uuidv4() }));

export const MacroProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const { sendCommand, addCommand } = useCommand();

  const [macros, setMacros] = useState<Macro[]>([]);
  const [isLoaded, setIsLoaded] = useState(false);
  const [runtime, setRuntime] = useState<RuntimeState>({ runningId: null, stepIndex: -1, startedAt: null });
  const [stopRequested, setStopRequested] = useState(false);

  const runtimeRef = useRef(runtime);
  const stopSignalRef = useRef<{
    shouldStop: boolean;
    timer: ReturnType<typeof setTimeout> | null;
    wake: (() => void) | null;
  }>({ shouldStop: false, timer: null, wake: null });

  runtimeRef.current = runtime;

  useEffect(() => {
    if (typeof window === 'undefined') return;
    try {
      const stored = window.localStorage.getItem(STORAGE_KEY);
      if (stored) {
        const parsed: Macro[] = JSON.parse(stored);
        if (Array.isArray(parsed)) {
          setMacros(parsed);
        }
      }
    } catch (error) {
      console.warn('[MacroProvider] Failed to load macros from storage', error);
    } finally {
      setIsLoaded(true);
    }
  }, []);

  useEffect(() => {
    if (!isLoaded || typeof window === 'undefined') return;
    try {
      window.localStorage.setItem(STORAGE_KEY, JSON.stringify(macros));
    } catch (error) {
      console.warn('[MacroProvider] Failed to persist macros', error);
    }
  }, [macros, isLoaded]);

  useEffect(
    () => () => {
      // Cleanup on unmount
      const signal = stopSignalRef.current;
      signal.shouldStop = true;
      if (signal.timer) clearTimeout(signal.timer);
      signal.timer = null;
      if (signal.wake) signal.wake();
      signal.wake = null;
    },
    [],
  );

  const createMacro = useCallback(
    (name = 'New Macro', description?: string) => {
      const now = Date.now();
      const macro: Macro = {
        id: uuidv4(),
        name,
        description,
        steps: [],
        createdAt: now,
        updatedAt: now,
      };
      setMacros((prev) => [...prev, macro]);
      return macro;
    },
    [],
  );

  const updateMacro = useCallback<MacroContextType['updateMacro']>((id, updates) => {
    setMacros((prev) =>
      prev.map((macro) => {
        if (macro.id !== id) return macro;
        return {
          ...macro,
          ...updates,
          steps: updates.steps ?? macro.steps,
          updatedAt: Date.now(),
        };
      }),
    );
  }, []);

  const deleteMacro = useCallback((id: string) => {
    setMacros((prev) => prev.filter((macro) => macro.id !== id));
  }, []);

  const duplicateMacro = useCallback(
    (id: string) => {
      const macro = macros.find((m) => m.id === id);
      if (!macro) return undefined;
      const now = Date.now();
      const clone: Macro = {
        ...macro,
        id: uuidv4(),
        name: `${macro.name || 'Macro'} Copy`,
        steps: cloneSteps(macro.steps),
        createdAt: now,
        updatedAt: now,
      };
      setMacros((prev) => [...prev, clone]);
      return clone;
    },
    [macros],
  );

  const addStep = useCallback<MacroContextType['addStep']>((macroId, step) => {
    setMacros((prev) =>
      prev.map((macro) => {
        if (macro.id !== macroId) return macro;
        return {
          ...macro,
          steps: [...macro.steps, step],
          updatedAt: Date.now(),
        };
      }),
    );
  }, []);

  const updateStep = useCallback<MacroContextType['updateStep']>((macroId, stepId, updates) => {
    setMacros((prev) =>
      prev.map((macro) => {
        if (macro.id !== macroId) return macro;
        const steps = macro.steps.map((step) =>
          step.id === stepId
            ? {
                ...step,
                ...updates,
              }
            : step,
        );
        return {
          ...macro,
          steps,
          updatedAt: Date.now(),
        };
      }),
    );
  }, []);

  const removeStep = useCallback<MacroContextType['removeStep']>((macroId, stepId) => {
    setMacros((prev) =>
      prev.map((macro) => {
        if (macro.id !== macroId) return macro;
        return {
          ...macro,
          steps: macro.steps.filter((step) => step.id !== stepId),
          updatedAt: Date.now(),
        };
      }),
    );
  }, []);

  const moveStep = useCallback<MacroContextType['moveStep']>((macroId, fromIndex, toIndex) => {
    setMacros((prev) =>
      prev.map((macro) => {
        if (macro.id !== macroId) return macro;
        const steps = [...macro.steps];
        if (fromIndex < 0 || fromIndex >= steps.length || toIndex < 0 || toIndex >= steps.length) {
          return macro;
        }
        const [item] = steps.splice(fromIndex, 1);
        steps.splice(toIndex, 0, item);
        return {
          ...macro,
          steps,
          updatedAt: Date.now(),
        };
      }),
    );
  }, []);

  const waitFor = useCallback((ms: number) => {
    if (ms <= 0) return Promise.resolve();
    return new Promise<void>((resolve) => {
      const timer = setTimeout(() => {
        const signal = stopSignalRef.current;
        signal.timer = null;
        signal.wake = null;
        resolve();
      }, ms);
      const signal = stopSignalRef.current;
      signal.timer = timer;
      signal.wake = resolve;
    });
  }, []);

  const stopMacro = useCallback(() => {
    const signal = stopSignalRef.current;
    if (!runtimeRef.current.runningId) {
      setStopRequested(false);
      return;
    }
    if (!signal.shouldStop) {
      addCommand('Macro stop requested');
    }
    signal.shouldStop = true;
    setStopRequested(true);
    if (signal.timer) {
      clearTimeout(signal.timer);
      signal.timer = null;
    }
    if (signal.wake) {
      signal.wake();
      signal.wake = null;
    }
  }, [addCommand]);

  const runMacro = useCallback<MacroContextType['runMacro']>(
    async (id) => {
      const macro = macros.find((m) => m.id === id);
      if (!macro) {
        throw new Error('Macro not found');
      }
      if (runtimeRef.current.runningId && runtimeRef.current.runningId !== id) {
        const message = 'Another macro is already running';
        addCommand(message);
        throw new Error(message);
      }
      if (macro.steps.length === 0) {
        addCommand(`Macro "${macro.name}" has no steps to run`);
        return;
      }

      stopSignalRef.current.shouldStop = false;
      stopSignalRef.current.wake = null;
      setStopRequested(false);

      addCommand(`Macro "${macro.name}" starting (${macro.steps.length} steps)`);
      setRuntime({ runningId: id, stepIndex: -1, startedAt: Date.now() });

      for (let index = 0; index < macro.steps.length; index += 1) {
        if (stopSignalRef.current.shouldStop) break;
        const step = macro.steps[index];
        setRuntime((prev) => ({ ...prev, stepIndex: index }));
        addCommand(
          `Macro step ${index + 1}/${macro.steps.length}: ${step.label || step.command}`,
        );

        try {
          sendCommand(step.command, step.payload);
        } catch (error) {
          console.error('[MacroProvider] Failed to send command', error);
          addCommand(`Macro command failed: ${step.command}`);
        }

        if (stopSignalRef.current.shouldStop) break;
        const delay = Math.max(0, step.delayMs ?? 0);
        await waitFor(delay);
      }

      const wasStopped = stopSignalRef.current.shouldStop;
      const macroName = macro.name || 'Macro';

      stopSignalRef.current.shouldStop = false;
      if (stopSignalRef.current.timer) {
        clearTimeout(stopSignalRef.current.timer);
        stopSignalRef.current.timer = null;
      }
      stopSignalRef.current.wake = null;

      setRuntime({ runningId: null, stepIndex: -1, startedAt: null });
      setStopRequested(false);

      addCommand(`Macro "${macroName}" ${wasStopped ? 'stopped' : 'completed'}`);
    },
    [addCommand, macros, sendCommand, waitFor],
  );

  const value = useMemo<MacroContextType>(
    () => ({
      macros,
      isLoaded,
      createMacro,
      updateMacro,
      deleteMacro,
      duplicateMacro,
      addStep,
      updateStep,
      removeStep,
      moveStep,
      runMacro,
      stopMacro,
      runtime,
      isRunning: runtime.runningId !== null,
      stopRequested,
    }),
    [
      addStep,
      createMacro,
      deleteMacro,
      duplicateMacro,
      isLoaded,
      macros,
      moveStep,
      runMacro,
      runtime,
      stopMacro,
      stopRequested,
      updateMacro,
      updateStep,
      removeStep,
    ],
  );

  return <MacroContext.Provider value={value}>{children}</MacroContext.Provider>;
};

