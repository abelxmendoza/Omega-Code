import { combineReducers } from '@reduxjs/toolkit';
import controlPanelReducer from './controlPanelReducer';

const rootReducer = combineReducers({
  controlPanel: controlPanelReducer,
});

export default rootReducer;
