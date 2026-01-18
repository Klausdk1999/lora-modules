import 'dotenv/config';
import express from 'express';
import path from 'path';
import type { Config } from './types';
import { initDatabase } from './database';
import { connectMqtt, disconnectMqtt } from './mqtt-client';
import routes from './routes';

// Load configuration from environment
function loadConfig(): Config {
  const broker = process.env.TTN_MQTT_BROKER;
  const username = process.env.TTN_USERNAME;
  const password = process.env.TTN_PASSWORD;

  if (!broker || !username || !password) {
    console.error('Missing required environment variables:');
    if (!broker) console.error('  - TTN_MQTT_BROKER');
    if (!username) console.error('  - TTN_USERNAME');
    if (!password) console.error('  - TTN_PASSWORD');
    process.exit(1);
  }

  return {
    mqtt: { broker, username, password },
    server: { port: parseInt(process.env.PORT || '3000', 10) },
    database: { path: process.env.DB_PATH || './data/ttn-data.db' },
  };
}

async function main() {
  console.log('='.repeat(60));
  console.log('TTN Data Collector - River Monitoring');
  console.log('='.repeat(60));

  // Load config
  const config = loadConfig();
  console.log('\n[Config] Loaded configuration');

  // Resolve database path relative to project root
  const dbPath = path.isAbsolute(config.database.path)
    ? config.database.path
    : path.join(__dirname, '..', config.database.path);

  // Initialize database
  await initDatabase(dbPath);

  // Connect to TTN MQTT
  connectMqtt(config);

  // Start Express server
  const app = express();
  app.use(express.json());
  app.use('/api', routes);

  // Root endpoint
  app.get('/', (_req, res) => {
    res.json({
      name: 'TTN Data Collector',
      description: 'Collects LoRaWAN uplinks from The Things Network',
      endpoints: {
        health: '/api/health',
        stats: '/api/stats',
        devices: '/api/devices',
        uplinks: '/api/uplinks',
        deviceLatest: '/api/devices/:deviceId/latest',
        deviceUplinks: '/api/devices/:deviceId/uplinks',
      },
    });
  });

  app.listen(config.server.port, () => {
    console.log(`[HTTP] Server listening on port ${config.server.port}`);
    console.log(`[HTTP] API available at http://localhost:${config.server.port}/api`);
    console.log('\n' + '='.repeat(60));
    console.log('Waiting for uplinks from TTN...');
    console.log('='.repeat(60) + '\n');
  });

  // Graceful shutdown
  process.on('SIGINT', () => {
    console.log('\n[Shutdown] Received SIGINT, shutting down...');
    disconnectMqtt();
    process.exit(0);
  });

  process.on('SIGTERM', () => {
    console.log('\n[Shutdown] Received SIGTERM, shutting down...');
    disconnectMqtt();
    process.exit(0);
  });
}

main().catch((error) => {
  console.error('Fatal error:', error);
  process.exit(1);
});
