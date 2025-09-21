// src/components/common/StatusDot.tsx
'use client';
import React from 'react';
import { ServerStatus, statusColor } from '@/constants/status';

export default function StatusDot({
  status,
  title,
  size = 8, // px
}: {
  status: ServerStatus;
  title?: string;
  size?: number;
}) {
  return (
    <span
      className={`inline-block rounded-full ${statusColor[status]}`}
      style={{ width: size, height: size }}
      title={title}
      aria-label={title}
    />
  );
}

