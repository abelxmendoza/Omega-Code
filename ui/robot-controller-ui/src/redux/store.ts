// File: /src/redux/store.ts
import { configureStore } from '@reduxjs/toolkit';
import controlPanelReducer from './reducers/controlPanelReducer';
import commandLogReducer from './reducers/commandLogReducer';

const store = configureStore({
  reducer: {
    controlPanel: controlPanelReducer,
    commands: commandLogReducer,
  },
});

export type RootState = ReturnType<typeof store.getState>;
export type AppDispatch = typeof store.dispatch;

export default store;
