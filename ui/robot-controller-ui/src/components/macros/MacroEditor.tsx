import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { v4 as uuidv4 } from 'uuid';
import {
  DEFAULT_MACRO_COMMAND,
  MACRO_COMMAND_GROUPS,
  getMacroCommandLabel,
} from '@/constants/macroCommands';
import { Macro, MacroStep, useMacro } from '@/context/MacroContext';

interface MacroEditorProps {
  macro: Macro;
}

type DraftState = {
  text: string;
  error?: string;
};

const fieldLabel = 'block text-sm font-medium text-gray-700 mb-1';
const inputBase =
  'w-full rounded-md border border-gray-300 bg-white px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-200';

const iconButtonClasses =
  'inline-flex items-center justify-center rounded-md border border-gray-300 bg-white px-2 py-1 text-xs font-medium text-gray-700 shadow-sm hover:bg-gray-50 disabled:opacity-40 disabled:cursor-not-allowed';

const dangerButtonClasses =
  'inline-flex items-center justify-center rounded-md border border-red-200 bg-red-50 px-2 py-1 text-xs font-medium text-red-600 shadow-sm hover:bg-red-100 disabled:opacity-40 disabled:cursor-not-allowed';

const addButtonClasses =
  'inline-flex items-center justify-center rounded-md border border-indigo-200 bg-indigo-50 px-3 py-2 text-sm font-medium text-indigo-700 shadow-sm hover:bg-indigo-100';

const MacroEditor: React.FC<MacroEditorProps> = ({ macro }) => {
  const { updateMacro, addStep, updateStep, removeStep, moveStep, runtime, isRunning } = useMacro();

  const [payloadDrafts, setPayloadDrafts] = useState<Record<string, DraftState>>({});
  const lastMacroIdRef = useRef<string | null>(null);

  const descriptionLookup = useMemo(() => {
    const map = new Map<string, string | undefined>();
    for (const group of MACRO_COMMAND_GROUPS) {
      for (const option of group.options) {
        map.set(option.value, option.description);
      }
    }
    return map;
  }, []);

  useEffect(() => {
    if (lastMacroIdRef.current === macro.id) {
      return;
    }
    lastMacroIdRef.current = macro.id;
    const initial: Record<string, DraftState> = {};
    macro.steps.forEach((step) => {
      initial[step.id] = {
        text: step.payload ? JSON.stringify(step.payload, null, 2) : '',
        error: undefined,
      };
    });
    setPayloadDrafts(initial);
  }, [macro.id, macro.steps]);

  useEffect(() => {
    setPayloadDrafts((prev) => {
      const next: Record<string, DraftState> = {};
      macro.steps.forEach((step) => {
        next[step.id] = prev[step.id] ?? {
          text: step.payload ? JSON.stringify(step.payload, null, 2) : '',
          error: undefined,
        };
      });
      return next;
    });
  }, [macro.steps]);

  const handleNameChange = useCallback(
    (event: React.ChangeEvent<HTMLInputElement>) => {
      updateMacro(macro.id, { name: event.target.value });
    },
    [macro.id, updateMacro],
  );

  const handleDescriptionChange = useCallback(
    (event: React.ChangeEvent<HTMLTextAreaElement>) => {
      updateMacro(macro.id, { description: event.target.value });
    },
    [macro.id, updateMacro],
  );

  const handleAddStep = useCallback(() => {
    const newStep: MacroStep = {
      id: uuidv4(),
      command: DEFAULT_MACRO_COMMAND,
      delayMs: 0,
    };
    addStep(macro.id, newStep);
  }, [addStep, macro.id]);

  const handleLabelChange = useCallback(
    (stepId: string, value: string) => {
      updateStep(macro.id, stepId, { label: value || undefined });
    },
    [macro.id, updateStep],
  );

  const handleCommandChange = useCallback(
    (stepId: string, value: string) => {
      updateStep(macro.id, stepId, { command: value });
    },
    [macro.id, updateStep],
  );

  const handleDelayChange = useCallback(
    (stepId: string, value: string) => {
      const parsed = Number(value);
      const sanitized = Number.isFinite(parsed) ? Math.max(0, Math.round(parsed)) : 0;
      updateStep(macro.id, stepId, { delayMs: sanitized });
    },
    [macro.id, updateStep],
  );

  const handlePayloadChange = useCallback(
    (stepId: string, text: string) => {
      setPayloadDrafts((prev) => ({
        ...prev,
        [stepId]: {
          text,
          error: prev[stepId]?.error,
        },
      }));

      if (text.trim() === '') {
        updateStep(macro.id, stepId, { payload: undefined });
        setPayloadDrafts((prev) => ({
          ...prev,
          [stepId]: { text: '', error: undefined },
        }));
        return;
      }

      try {
        const parsed = JSON.parse(text);
        updateStep(macro.id, stepId, { payload: parsed });
        setPayloadDrafts((prev) => ({
          ...prev,
          [stepId]: { text, error: undefined },
        }));
      } catch {
        setPayloadDrafts((prev) => ({
          ...prev,
          [stepId]: { text, error: 'Invalid JSON payload' },
        }));
      }
    },
    [macro.id, updateStep],
  );

  const handleRemoveStep = useCallback(
    (stepId: string) => {
      const step = macro.steps.find((s) => s.id === stepId);
      const label = step?.label || getMacroCommandLabel(step?.command || '');
      const shouldDelete =
        typeof window === 'undefined'
          ? true
          : window.confirm(`Remove step "${label}"?`);
      if (!shouldDelete) return;
      removeStep(macro.id, stepId);
      setPayloadDrafts((prev) => {
        const { [stepId]: _removed, ...rest } = prev;
        return rest;
      });
    },
    [macro.id, macro.steps, removeStep],
  );

  const handleMoveStep = useCallback(
    (currentIndex: number, direction: -1 | 1) => {
      const nextIndex = currentIndex + direction;
      moveStep(macro.id, currentIndex, nextIndex);
    },
    [macro.id, moveStep],
  );

  const activeStepIndex = useMemo(() => {
    if (runtime.runningId !== macro.id) return -1;
    return runtime.stepIndex;
  }, [macro.id, runtime]);

  return (
    <div className="space-y-6">
      <div>
        <label className={fieldLabel} htmlFor={`macro-name-${macro.id}`}>
          Macro Name
        </label>
        <input
          id={`macro-name-${macro.id}`}
          className={inputBase}
          value={macro.name}
          placeholder="Untitled macro"
          onChange={handleNameChange}
        />
      </div>

      <div>
        <label className={fieldLabel} htmlFor={`macro-description-${macro.id}`}>
          Notes (optional)
        </label>
        <textarea
          id={`macro-description-${macro.id}`}
          className={`${inputBase} min-h-[72px]`}
          value={macro.description ?? ''}
          placeholder="Describe what this macro does for future you…"
          onChange={handleDescriptionChange}
        />
      </div>

      <div className="flex items-center justify-between">
        <h3 className="text-base font-semibold text-gray-900">Steps</h3>
        <button type="button" className={addButtonClasses} onClick={handleAddStep}>
          + Add Step
        </button>
      </div>

      {macro.steps.length === 0 ? (
        <p className="text-sm text-gray-500">
          No steps yet. Add steps to orchestrate a sequence of commands.
        </p>
      ) : (
        <div className="space-y-4">
          {macro.steps.map((step, index) => {
            const draft = payloadDrafts[step.id] ?? { text: '' };
            const isActive = activeStepIndex === index && isRunning;
            const isDone = isRunning && runtime.runningId === macro.id && runtime.stepIndex > index;
            return (
              <div
                key={step.id}
                className={`rounded-xl border bg-white p-4 shadow-sm transition ${
                  isActive
                    ? 'border-indigo-400 ring-2 ring-indigo-200'
                    : isDone
                    ? 'border-emerald-200'
                    : 'border-gray-200'
                }`}
              >
                <div className="flex flex-wrap items-center justify-between gap-2">
                  <div className="text-sm font-semibold text-gray-800">
                    Step {index + 1}
                    {step.label ? ` · ${step.label}` : ''}
                  </div>
                  <div className="flex items-center gap-2">
                    <button
                      type="button"
                      className={iconButtonClasses}
                      onClick={() => handleMoveStep(index, -1)}
                      disabled={index === 0}
                    >
                      ↑
                    </button>
                    <button
                      type="button"
                      className={iconButtonClasses}
                      onClick={() => handleMoveStep(index, 1)}
                      disabled={index === macro.steps.length - 1}
                    >
                      ↓
                    </button>
                    <button
                      type="button"
                      className={dangerButtonClasses}
                      onClick={() => handleRemoveStep(step.id)}
                    >
                      Remove
                    </button>
                  </div>
                </div>

                <div className="mt-3 grid gap-4 md:grid-cols-2">
                  <div>
                    <label className={fieldLabel} htmlFor={`macro-${macro.id}-step-${step.id}-label`}>
                      Friendly label (optional)
                    </label>
                    <input
                      id={`macro-${macro.id}-step-${step.id}-label`}
                      className={inputBase}
                      value={step.label ?? ''}
                      placeholder="e.g. Move off the line"
                      onChange={(event) => handleLabelChange(step.id, event.target.value)}
                    />
                  </div>
                  <div>
                    <label className={fieldLabel} htmlFor={`macro-${macro.id}-step-${step.id}-command`}>
                      Command
                    </label>
                    <select
                      id={`macro-${macro.id}-step-${step.id}-command`}
                      className={inputBase}
                      value={step.command}
                      onChange={(event) => handleCommandChange(step.id, event.target.value)}
                    >
                      {MACRO_COMMAND_GROUPS.map((group) => (
                        <optgroup key={group.group} label={group.group}>
                          {group.options.map((option) => (
                            <option key={option.value} value={option.value}>
                              {option.label}
                            </option>
                          ))}
                        </optgroup>
                      ))}
                    </select>
                  </div>
                </div>

                <div className="mt-4 grid gap-4 md:grid-cols-2">
                  <div>
                    <label className={fieldLabel} htmlFor={`macro-${macro.id}-step-${step.id}-delay`}>
                      Delay after command (ms)
                    </label>
                    <input
                      id={`macro-${macro.id}-step-${step.id}-delay`}
                      className={inputBase}
                      type="number"
                      min={0}
                      step={50}
                      value={step.delayMs ?? 0}
                      onChange={(event) => handleDelayChange(step.id, event.target.value)}
                    />
                    <p className="mt-1 text-xs text-gray-500">
                      Wait before the next step. Use 0 for immediate execution.
                    </p>
                  </div>
                  <div>
                    <label className={fieldLabel} htmlFor={`macro-${macro.id}-step-${step.id}-payload`}>
                      Payload (JSON)
                    </label>
                    <textarea
                      id={`macro-${macro.id}-step-${step.id}-payload`}
                      className={`${inputBase} min-h-[96px] font-mono text-xs`}
                      value={draft.text}
                      onChange={(event) => handlePayloadChange(step.id, event.target.value)}
                      placeholder="{ &quot;value&quot;: 2048 }"
                    />
                    {draft.error ? (
                      <p className="mt-1 text-xs text-red-600">{draft.error}</p>
                    ) : (
                      <p className="mt-1 text-xs text-gray-500">
                        Leave blank for commands without parameters.
                      </p>
                    )}
                    {descriptionLookup.get(step.command) ? (
                      <p className="mt-1 text-xs text-gray-400">
                        {descriptionLookup.get(step.command)}
                      </p>
                    ) : null}
                  </div>
                </div>
              </div>
            );
          })}
        </div>
      )}
    </div>
  );
};

export default MacroEditor;

