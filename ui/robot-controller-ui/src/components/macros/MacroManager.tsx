import React, { useCallback, useEffect, useMemo, useState } from 'react';
import MacroEditor from './MacroEditor';
import { useMacro } from '@/context/MacroContext';
import { getMacroCommandLabel } from '@/constants/macroCommands';
import { useCommand } from '@/context/CommandContext';

const listButtonBase =
  'w-full rounded-xl border px-4 py-3 text-left shadow-sm transition focus:outline-none focus:ring-2 focus:ring-indigo-300';

const actionButton =
  'inline-flex items-center justify-center rounded-md border border-gray-300 bg-white px-3 py-1 text-sm font-medium text-gray-700 shadow-sm hover:bg-gray-50 disabled:opacity-40 disabled:cursor-not-allowed';

const primaryButton =
  'inline-flex items-center justify-center rounded-md border border-indigo-600 bg-indigo-600 px-4 py-2 text-sm font-semibold text-white shadow-sm hover:bg-indigo-500 disabled:opacity-50';

const secondaryButton =
  'inline-flex items-center justify-center rounded-md border border-gray-200 bg-white px-4 py-2 text-sm font-medium text-gray-700 shadow-sm hover:bg-gray-100 disabled:opacity-40';

const dangerButton =
  'inline-flex items-center justify-center rounded-md border border-red-200 bg-red-50 px-3 py-1 text-sm font-medium text-red-600 shadow-sm hover:bg-red-100';

const MacroManager: React.FC = () => {
  const {
    macros,
    isLoaded,
    createMacro,
    deleteMacro,
    duplicateMacro,
    runMacro,
    stopMacro,
    runtime,
    isRunning,
    stopRequested,
  } = useMacro();
  const { addCommand } = useCommand();

  const [selectedId, setSelectedId] = useState<string | null>(null);

  useEffect(() => {
    if (macros.length === 0) {
      setSelectedId(null);
      return;
    }
    if (!selectedId || !macros.some((macro) => macro.id === selectedId)) {
      setSelectedId(macros[0].id);
    }
  }, [macros, selectedId]);

  const selectedMacro = useMemo(
    () => macros.find((macro) => macro.id === selectedId) ?? null,
    [macros, selectedId],
  );

  const runningMacro = useMemo(
    () => (runtime.runningId ? macros.find((macro) => macro.id === runtime.runningId) : undefined),
    [macros, runtime.runningId],
  );

  const formatTimestamp = useCallback((timestamp: number) => {
    try {
      return new Intl.DateTimeFormat(undefined, {
        hour: '2-digit',
        minute: '2-digit',
        month: 'short',
        day: 'numeric',
      }).format(new Date(timestamp));
    } catch {
      return new Date(timestamp).toLocaleString();
    }
  }, []);

  const handleCreate = useCallback(() => {
    const macro = createMacro(`Macro ${macros.length + 1}`);
    addCommand(`Macro created: ${macro.name}`);
    setSelectedId(macro.id);
  }, [addCommand, createMacro, macros.length]);

  const handleRun = useCallback(
    (macroId: string) => {
      runMacro(macroId).catch((error) => {
        const message = error instanceof Error ? error.message : String(error);
        addCommand(`Macro failed to start: ${message}`);
      });
    },
    [addCommand, runMacro],
  );

  const handleDuplicate = useCallback(
    (macroId: string) => {
      const copy = duplicateMacro(macroId);
      if (copy) {
        addCommand(`Macro duplicated: ${copy.name}`);
        setSelectedId(copy.id);
      }
    },
    [addCommand, duplicateMacro],
  );

  const handleDelete = useCallback(
    (macroId: string) => {
      const macro = macros.find((m) => m.id === macroId);
      const name = macro?.name || 'Macro';
      const shouldDelete =
        typeof window === 'undefined' ? true : window.confirm(`Delete "${name}"? This cannot be undone.`);
      if (!shouldDelete) return;
      deleteMacro(macroId);
      addCommand(`Macro deleted: ${name}`);
      if (selectedId === macroId) {
        setSelectedId(null);
      }
    },
    [addCommand, deleteMacro, macros, selectedId],
  );

  const recentCommand = useMemo(() => {
    if (!runningMacro || runtime.stepIndex < 0) return null;
    const step = runningMacro.steps[runtime.stepIndex];
    if (!step) return null;
    return step.label || getMacroCommandLabel(step.command);
  }, [runningMacro, runtime.stepIndex]);

  if (!isLoaded) {
    return (
      <section className="mx-auto w-full max-w-6xl rounded-2xl border border-gray-200 bg-white/80 p-6 shadow-sm">
        <h2 className="text-lg font-semibold text-gray-800">Macro Automation</h2>
        <p className="mt-2 text-sm text-gray-500">Loading saved macros…</p>
      </section>
    );
  }

  return (
    <section className="mx-auto w-full max-w-6xl rounded-2xl border border-gray-200 bg-white/90 p-6 shadow-lg backdrop-blur-sm">
      <div className="flex flex-col gap-4">
        <div className="flex flex-col gap-3 lg:flex-row lg:items-center lg:justify-between">
          <div>
            <h2 className="text-xl font-bold text-gray-900">Macro Automation (Beta)</h2>
            <p className="text-sm text-gray-600">
              Orchestrate repeatable command sequences without writing ROS nodes. Macros run through the
              standard WebSocket interface, so they work anywhere the UI does.
            </p>
          </div>
          <div className="flex flex-wrap items-center gap-2">
            {isRunning ? (
              <button type="button" className={secondaryButton} onClick={stopMacro}>
                Stop macro
              </button>
            ) : null}
            <button type="button" className={primaryButton} onClick={handleCreate}>
              New macro
            </button>
          </div>
        </div>

        {isRunning && runningMacro ? (
          <div className="rounded-xl border border-indigo-200 bg-indigo-50 p-4 text-sm text-indigo-900 shadow-inner">
            <div className="font-semibold">
              Running “{runningMacro.name}” — step {runtime.stepIndex + 1} of {runningMacro.steps.length}
            </div>
            {recentCommand ? <div className="mt-1">Current command: {recentCommand}</div> : null}
            {stopRequested ? (
              <div className="mt-1 text-xs text-indigo-700">Stop requested — finishing current step…</div>
            ) : null}
          </div>
        ) : null}

        <div className="grid gap-6 lg:grid-cols-[minmax(240px,280px)_1fr]">
          <div className="space-y-3">
            {macros.length === 0 ? (
              <div className="rounded-xl border border-dashed border-gray-300 bg-gray-50 p-4 text-sm text-gray-500">
                No macros yet. Create one to start chaining commands.
              </div>
            ) : (
              macros.map((macro) => {
                const isSelected = macro.id === selectedId;
                return (
                  <div
                    key={macro.id}
                    className={`${
                      isSelected
                        ? 'border-indigo-300 bg-indigo-50/80'
                        : 'border-gray-200 bg-white'
                    } ${listButtonBase}`}
                    onClick={() => setSelectedId(macro.id)}
                    role="button"
                    tabIndex={0}
                    onKeyDown={(event) => {
                      if (event.key === 'Enter' || event.key === ' ') {
                        event.preventDefault();
                        setSelectedId(macro.id);
                      }
                    }}
                  >
                    <div className="flex items-start justify-between gap-2">
                      <div>
                        <div className="text-base font-semibold text-gray-900">{macro.name || 'Untitled macro'}</div>
                        <div className="mt-1 text-xs text-gray-500">
                          {macro.steps.length} step{macro.steps.length === 1 ? '' : 's'} · Updated {formatTimestamp(macro.updatedAt)}
                        </div>
                      </div>
                      <div className="flex flex-col gap-1 text-xs text-gray-400">
                        <button
                          type="button"
                          className={`${actionButton} px-2 py-1`}
                          onClick={(event) => {
                            event.stopPropagation();
                            handleRun(macro.id);
                          }}
                          disabled={isRunning && runtime.runningId !== macro.id}
                        >
                          Run
                        </button>
                        <button
                          type="button"
                          className={`${actionButton} px-2 py-1`}
                          onClick={(event) => {
                            event.stopPropagation();
                            handleDuplicate(macro.id);
                          }}
                        >
                          Duplicate
                        </button>
                        <button
                          type="button"
                          className={`${dangerButton} px-2 py-1`}
                          onClick={(event) => {
                            event.stopPropagation();
                            handleDelete(macro.id);
                          }}
                        >
                          Delete
                        </button>
                      </div>
                    </div>
                  </div>
                );
              })
            )}
          </div>

          <div className="min-h-[320px] rounded-2xl border border-gray-200 bg-white p-5 shadow-inner">
            {selectedMacro ? (
              <MacroEditor macro={selectedMacro} />
            ) : macros.length === 0 ? (
              <div className="flex h-full flex-col items-center justify-center text-center text-sm text-gray-500">
                Create a macro on the left to start building automation sequences.
              </div>
            ) : (
              <div className="flex h-full flex-col items-center justify-center text-center text-sm text-gray-500">
                Select a macro to edit its steps.
              </div>
            )}
          </div>
        </div>
      </div>
    </section>
  );
};

export default MacroManager;

