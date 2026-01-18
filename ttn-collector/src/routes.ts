import { Router, Request, Response } from 'express';
import { getUplinks, getDevices, getLatestUplink, getStats } from './database';
import { getMqttClient } from './mqtt-client';

const router = Router();

// Helper to safely get query param as string
function getQueryString(value: unknown): string | undefined {
  if (typeof value === 'string') return value;
  if (Array.isArray(value) && typeof value[0] === 'string') return value[0];
  return undefined;
}

// Health check
router.get('/health', (_req: Request, res: Response) => {
  const mqttClient = getMqttClient();
  res.json({
    status: 'ok',
    mqtt: mqttClient?.connected ? 'connected' : 'disconnected',
    timestamp: new Date().toISOString(),
  });
});

// Get database stats
router.get('/stats', (_req: Request, res: Response) => {
  try {
    const stats = getStats();
    res.json(stats);
  } catch (error) {
    res.status(500).json({ error: 'Failed to get stats' });
  }
});

// List all devices
router.get('/devices', (_req: Request, res: Response) => {
  try {
    const devices = getDevices();
    res.json(devices);
  } catch (error) {
    res.status(500).json({ error: 'Failed to get devices' });
  }
});

// Get latest uplink for a device
router.get('/devices/:deviceId/latest', (req: Request, res: Response) => {
  try {
    const deviceId = req.params.deviceId as string;
    const uplink = getLatestUplink(deviceId);

    if (!uplink) {
      res.status(404).json({ error: 'Device not found' });
      return;
    }

    // Add computed fields
    const response = {
      ...uplink,
      distance_cm: uplink.distance_mm / 10,
    };

    res.json(response);
  } catch (error) {
    res.status(500).json({ error: 'Failed to get latest uplink' });
  }
});

// Get uplinks (with optional filters)
router.get('/uplinks', (req: Request, res: Response) => {
  try {
    const deviceId = getQueryString(req.query.device_id);
    const limit = parseInt(getQueryString(req.query.limit) || '100', 10);
    const offset = parseInt(getQueryString(req.query.offset) || '0', 10);
    const startDate = getQueryString(req.query.start_date);
    const endDate = getQueryString(req.query.end_date);

    const uplinks = getUplinks({
      deviceId,
      limit,
      offset,
      startDate,
      endDate,
    });

    // Add computed fields
    const response = uplinks.map(uplink => ({
      ...uplink,
      distance_cm: uplink.distance_mm / 10,
    }));

    res.json(response);
  } catch (error) {
    res.status(500).json({ error: 'Failed to get uplinks' });
  }
});

// Get uplinks for a specific device
router.get('/devices/:deviceId/uplinks', (req: Request, res: Response) => {
  try {
    const deviceId = req.params.deviceId as string;
    const limit = parseInt(getQueryString(req.query.limit) || '100', 10);
    const offset = parseInt(getQueryString(req.query.offset) || '0', 10);
    const startDate = getQueryString(req.query.start_date);
    const endDate = getQueryString(req.query.end_date);

    const uplinks = getUplinks({
      deviceId,
      limit,
      offset,
      startDate,
      endDate,
    });

    const response = uplinks.map(uplink => ({
      ...uplink,
      distance_cm: uplink.distance_mm / 10,
    }));

    res.json(response);
  } catch (error) {
    res.status(500).json({ error: 'Failed to get uplinks' });
  }
});

export default router;
