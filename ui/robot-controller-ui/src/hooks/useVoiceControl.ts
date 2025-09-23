import { useState, useEffect, useRef, useCallback } from 'react';

interface VoiceControlOptions {
  onCommand?: (command: string) => void;
  language?: string;
  continuous?: boolean;
}

interface VoiceControlState {
  isListening: boolean;
  isSupported: boolean;
  transcript: string;
  error: string | null;
}

export const useVoiceControl = (options: VoiceControlOptions = {}) => {
  const { onCommand, language = 'en-US', continuous = true } = options;
  
  const [state, setState] = useState<VoiceControlState>({
    isListening: false,
    isSupported: false,
    transcript: '',
    error: null
  });

  const recognitionRef = useRef<any>(null);

  const startListening = useCallback(() => {
    if (!recognitionRef.current) return;

    try {
      recognitionRef.current.start();
      setState(prev => ({ ...prev, isListening: true, error: null }));
    } catch (err) {
      setState(prev => ({ 
        ...prev, 
        error: 'Failed to start voice recognition',
        isListening: false 
      }));
    }
  }, []);

  const stopListening = useCallback(() => {
    if (recognitionRef.current) {
      recognitionRef.current.stop();
      setState(prev => ({ ...prev, isListening: false }));
    }
  }, []);

  const processCommand = useCallback((transcript: string) => {
    const command = transcript.toLowerCase().trim();
    
    // Map voice commands to robot commands
    const commandMap: Record<string, string> = {
      'forward': 'forward',
      'go forward': 'forward',
      'move forward': 'forward',
      'backward': 'backward',
      'go back': 'backward',
      'move back': 'backward',
      'left': 'left',
      'turn left': 'left',
      'go left': 'left',
      'right': 'right',
      'turn right': 'right',
      'go right': 'right',
      'stop': 'stop',
      'halt': 'stop',
      'pause': 'stop',
      'faster': 'speed_up',
      'slower': 'speed_down',
      'increase speed': 'speed_up',
      'decrease speed': 'speed_down',
      'camera up': 'camera_up',
      'camera down': 'camera_down',
      'look up': 'camera_up',
      'look down': 'camera_down',
      'look left': 'camera_left',
      'look right': 'camera_right'
    };

    const matchedCommand = commandMap[command];
    if (matchedCommand && onCommand) {
      onCommand(matchedCommand);
    }
  }, [onCommand]);

  useEffect(() => {
    // Check if speech recognition is supported
    const SpeechRecognition = (window as any).SpeechRecognition || (window as any).webkitSpeechRecognition;
    
    if (!SpeechRecognition) {
      setState(prev => ({ 
        ...prev, 
        isSupported: false, 
        error: 'Speech recognition not supported in this browser' 
      }));
      return;
    }

    setState(prev => ({ ...prev, isSupported: true }));

    const recognition = new SpeechRecognition();
    recognitionRef.current = recognition;

    recognition.continuous = continuous;
    recognition.interimResults = true;
    recognition.lang = language;

    recognition.onstart = () => {
      setState(prev => ({ ...prev, isListening: true, error: null }));
    };

    recognition.onresult = (event: any) => {
      let finalTranscript = '';
      
      for (let i = event.resultIndex; i < event.results.length; i++) {
        if (event.results[i].isFinal) {
          finalTranscript += event.results[i][0].transcript;
        }
      }

      if (finalTranscript) {
        setState(prev => ({ ...prev, transcript: finalTranscript }));
        processCommand(finalTranscript);
      }
    };

    recognition.onerror = (event: any) => {
      setState(prev => ({ 
        ...prev, 
        error: `Speech recognition error: ${event.error}`,
        isListening: false 
      }));
    };

    recognition.onend = () => {
      setState(prev => ({ ...prev, isListening: false }));
    };

    return () => {
      if (recognitionRef.current) {
        recognitionRef.current.stop();
      }
    };
  }, [language, continuous, processCommand]);

  return {
    ...state,
    startListening,
    stopListening,
    toggleListening: state.isListening ? stopListening : startListening
  };
};
