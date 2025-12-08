/**
 * Unit Tests for ProfileSelector Component
 */

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { renderWithProviders } from '../utils/test-helpers';
import { ProfileSelector } from '@/components/settings/ProfileSelector';
import * as useProfilesHook from '@/hooks/useProfiles';
import * as useConfigSectionHook from '@/hooks/useConfigSection';

jest.mock('@/hooks/useProfiles');
jest.mock('@/hooks/useConfigSection');
jest.mock('@/utils/env', () => ({
  ROBOT_ENABLED: true,
}));

describe('ProfileSelector', () => {
  const mockProfiles = {
    pi4b: {
      description: 'Raspberry Pi 4B',
      hardware: {
        cpu_cores: 4,
        ram_gb: 4,
      },
      capabilities: {
        ml_capable: false,
        slam_capable: false,
      },
      recommended_settings: {
        camera_backend: 'picamera2',
        camera_width: 640,
        camera_height: 480,
        movement_profile: 'smooth',
      },
    },
    jetson: {
      description: 'NVIDIA Jetson',
      hardware: {
        cpu_cores: 8,
        ram_gb: 8,
      },
      capabilities: {
        ml_capable: true,
        slam_capable: true,
      },
      recommended_settings: {
        camera_backend: 'picamera2',
        camera_width: 1920,
        camera_height: 1080,
        movement_profile: 'aggressive',
      },
    },
  };

  beforeEach(() => {
    jest.clearAllMocks();
    
    (useProfilesHook.useProfiles as jest.Mock).mockReturnValue({
      profiles: mockProfiles,
      activeProfile: 'pi4b',
      loading: false,
      error: null,
      getProfile: jest.fn(),
      refresh: jest.fn(),
    });

    (useConfigSectionHook.useConfigSection as jest.Mock).mockReturnValue({
      section: { profile: 'pi4b' },
      loading: false,
      error: null,
      updateSection: jest.fn().mockResolvedValue(true),
      refresh: jest.fn(),
    });
  });

  it('renders profile options', () => {
    renderWithProviders(<ProfileSelector />);
    
    expect(screen.getByText('pi4b')).toBeInTheDocument();
    expect(screen.getByText('jetson')).toBeInTheDocument();
  });

  it('displays profile details', () => {
    renderWithProviders(<ProfileSelector />);
    
    expect(screen.getByText('Raspberry Pi 4B')).toBeInTheDocument();
    expect(screen.getByText(/4 cores/i)).toBeInTheDocument();
    expect(screen.getByText(/4GB/i)).toBeInTheDocument();
  });

  it('shows selected profile', () => {
    renderWithProviders(<ProfileSelector currentProfile="pi4b" />);
    
    const pi4bButton = screen.getByText('pi4b').closest('button');
    expect(pi4bButton).toHaveClass('border-purple-500');
  });

  it('calls updateSection when profile is changed', async () => {
    const mockUpdateSection = jest.fn().mockResolvedValue(true);
    (useConfigSectionHook.useConfigSection as jest.Mock).mockReturnValue({
      section: { profile: 'pi4b' },
      loading: false,
      error: null,
      updateSection: mockUpdateSection,
      refresh: jest.fn(),
    });

    renderWithProviders(<ProfileSelector />);
    
    const jetsonButton = screen.getByText('jetson').closest('button');
    fireEvent.click(jetsonButton!);
    
    await waitFor(() => {
      expect(mockUpdateSection).toHaveBeenCalledWith({ profile: 'jetson' });
    });
  });

  it('shows loading state', () => {
    (useProfilesHook.useProfiles as jest.Mock).mockReturnValue({
      profiles: {},
      activeProfile: null,
      loading: true,
      error: null,
      getProfile: jest.fn(),
      refresh: jest.fn(),
    });

    renderWithProviders(<ProfileSelector />);
    
    expect(screen.getByText(/Loading profiles/i)).toBeInTheDocument();
  });
});

