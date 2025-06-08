/*
# File: /src/redux/actions/index.ts
# Summary:
Defines and exports all Redux action creators, organized for scalability.
*/

export const ActionTypes = {
    ADD_COMMAND: 'ADD_COMMAND',
    CLEAR_COMMANDS: 'CLEAR_COMMANDS',
    UPDATE_CONTROL_PANEL: 'UPDATE_CONTROL_PANEL',
    TOGGLE_MODAL: 'TOGGLE_MODAL',
  };
  
  // Action creators for command log management
  export const addCommand = (command: string) => {
    return {
      type: ActionTypes.ADD_COMMAND,
      payload: command,
    };
  };
  
  export const clearCommands = () => {
    return {
      type: ActionTypes.CLEAR_COMMANDS,
    };
  };
  
  // Action creators for control panel updates
  export const updateControlPanel = (data: any) => {
    return {
      type: ActionTypes.UPDATE_CONTROL_PANEL,
      payload: data,
    };
  };
  
  // Action creators for UI state management
  export const toggleModal = (isOpen: boolean) => {
    return {
      type: ActionTypes.TOGGLE_MODAL,
      payload: isOpen,
    };
  };
  