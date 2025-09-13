/*
# File: /Omega-Code/ui/robot-controller-ui/tests/test-utils.tsx
# Summary:
#   Testing helpers for RTL + Redux.
#   - Strongly typed store setup with optional preloadedState
#   - render() returns the RTL result plus { store } for assertions
#   - Compatible with older Redux/RTK (no PreloadedState import needed)
*/

import React, { PropsWithChildren, ReactElement } from 'react';
import { render as rtlRender, type RenderOptions } from '@testing-library/react';
import { Provider } from 'react-redux';
import { configureStore } from '@reduxjs/toolkit';
import rootReducer from '../src/redux/reducers'; // adjust if your path differs

/* ------------------------------ Types ------------------------------ */

export type RootState = ReturnType<typeof rootReducer>;
export type AppStore = ReturnType<typeof setupStore>;

/** Deep partial utility for older Redux/RTK where PreloadedState isn't exported. */
type DeepPartial<T> = {
  [K in keyof T]?: T[K] extends object ? DeepPartial<T[K]> : T[K];
};

/* -------------------------- Store factory -------------------------- */

function setupStore(preloadedState?: DeepPartial<RootState>) {
  return configureStore({
    reducer: rootReducer,
    // cast is safe for tests; slices will validate their own defaults
    preloadedState: preloadedState as any,
  });
}

/* ---------------------- Extended render options -------------------- */

interface ExtendedRenderOptions extends Omit<RenderOptions, 'wrapper'> {
  preloadedState?: DeepPartial<RootState>;
  store?: AppStore;
}

/* ----------------------------- render ------------------------------ */

function render(
  ui: ReactElement,
  {
    preloadedState,
    store = setupStore(preloadedState),
    ...renderOptions
  }: ExtendedRenderOptions = {}
) {
  function Wrapper({ children }: PropsWithChildren<{}>) {
    return <Provider store={store}>{children}</Provider>;
  }

  return {
    store,
    ...rtlRender(ui, { wrapper: Wrapper, ...renderOptions }),
  };
}

/* ----------------------------- Exports ----------------------------- */

export * from '@testing-library/react';
export { render, setupStore };
