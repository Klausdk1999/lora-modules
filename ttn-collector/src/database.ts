import initSqlJs, { Database } from 'sql.js';
import path from 'path';
import fs from 'fs';
import type { UplinkRecord } from './types';

let db: Database;
let dbPath: string;

export async function initDatabase(dbFilePath: string): Promise<Database> {
  dbPath = dbFilePath;

  // Ensure directory exists
  const dir = path.dirname(dbPath);
  if (!fs.existsSync(dir)) {
    fs.mkdirSync(dir, { recursive: true });
  }

  // Initialize sql.js
  const SQL = await initSqlJs();

  // Load existing database or create new one
  if (fs.existsSync(dbPath)) {
    const fileBuffer = fs.readFileSync(dbPath);
    db = new SQL.Database(fileBuffer);
    console.log(`[DB] Loaded existing database from ${dbPath}`);
  } else {
    db = new SQL.Database();
    console.log(`[DB] Created new database at ${dbPath}`);
  }

  // Create uplinks table
  db.run(`
    CREATE TABLE IF NOT EXISTS uplinks (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      device_id TEXT NOT NULL,
      dev_eui TEXT NOT NULL,
      received_at TEXT NOT NULL,
      f_port INTEGER NOT NULL,
      f_cnt INTEGER NOT NULL,
      raw_payload TEXT NOT NULL,
      sensor_type INTEGER NOT NULL,
      distance_mm INTEGER NOT NULL,
      signal_strength INTEGER NOT NULL,
      temperature INTEGER NOT NULL,
      battery_percent INTEGER NOT NULL,
      reading_count INTEGER NOT NULL,
      rssi INTEGER NOT NULL,
      snr REAL NOT NULL,
      gateway_id TEXT NOT NULL,
      spreading_factor INTEGER,
      frequency TEXT,
      created_at TEXT DEFAULT (datetime('now'))
    )
  `);

  // Create indexes
  db.run('CREATE INDEX IF NOT EXISTS idx_uplinks_device_id ON uplinks(device_id)');
  db.run('CREATE INDEX IF NOT EXISTS idx_uplinks_received_at ON uplinks(received_at)');
  db.run('CREATE INDEX IF NOT EXISTS idx_uplinks_dev_eui ON uplinks(dev_eui)');

  // Save to disk
  saveDatabase();

  return db;
}

function saveDatabase(): void {
  if (!db || !dbPath) return;
  const data = db.export();
  const buffer = Buffer.from(data);
  fs.writeFileSync(dbPath, buffer);
}

// Helper to sanitize values for sql.js (converts undefined to null)
function toSqlValue(value: unknown): string | number | null {
  if (value === undefined || value === null) return null;
  return value as string | number;
}

export function insertUplink(record: UplinkRecord): number {
  const stmt = db.prepare(`
    INSERT INTO uplinks (
      device_id, dev_eui, received_at, f_port, f_cnt, raw_payload,
      sensor_type, distance_mm, signal_strength, temperature,
      battery_percent, reading_count, rssi, snr, gateway_id,
      spreading_factor, frequency
    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
  `);

  stmt.run([
    toSqlValue(record.device_id),
    toSqlValue(record.dev_eui),
    toSqlValue(record.received_at),
    toSqlValue(record.f_port) ?? 0,
    toSqlValue(record.f_cnt) ?? 0,
    toSqlValue(record.raw_payload),
    toSqlValue(record.sensor_type),
    toSqlValue(record.distance_mm),
    toSqlValue(record.signal_strength),
    toSqlValue(record.temperature),
    toSqlValue(record.battery_percent),
    toSqlValue(record.reading_count),
    toSqlValue(record.rssi),
    toSqlValue(record.snr),
    toSqlValue(record.gateway_id),
    toSqlValue(record.spreading_factor),
    toSqlValue(record.frequency),
  ]);

  stmt.free();

  // Get last inserted row ID
  const result = db.exec('SELECT last_insert_rowid() as id');
  const lastId = result[0]?.values[0]?.[0] as number;

  // Save to disk after each insert
  saveDatabase();

  return lastId;
}

export function getUplinks(options: {
  deviceId?: string;
  limit?: number;
  offset?: number;
  startDate?: string;
  endDate?: string;
}): UplinkRecord[] {
  const { deviceId, limit = 100, offset = 0, startDate, endDate } = options;

  let query = 'SELECT * FROM uplinks WHERE 1=1';
  const params: (string | number)[] = [];

  if (deviceId) {
    query += ' AND device_id = ?';
    params.push(deviceId);
  }

  if (startDate) {
    query += ' AND received_at >= ?';
    params.push(startDate);
  }

  if (endDate) {
    query += ' AND received_at <= ?';
    params.push(endDate);
  }

  query += ' ORDER BY received_at DESC LIMIT ? OFFSET ?';
  params.push(limit, offset);

  const stmt = db.prepare(query);
  stmt.bind(params);

  const results: UplinkRecord[] = [];
  while (stmt.step()) {
    const row = stmt.getAsObject() as unknown as UplinkRecord;
    results.push(row);
  }
  stmt.free();

  return results;
}

export function getDevices(): Array<{ device_id: string; dev_eui: string; last_seen: string; uplink_count: number }> {
  const result = db.exec(`
    SELECT
      device_id,
      dev_eui,
      MAX(received_at) as last_seen,
      COUNT(*) as uplink_count
    FROM uplinks
    GROUP BY device_id
    ORDER BY last_seen DESC
  `);

  if (!result[0]) return [];

  const columns = result[0].columns;
  return result[0].values.map((row) => {
    const obj: Record<string, unknown> = {};
    columns.forEach((col, i) => {
      obj[col] = row[i];
    });
    return obj as { device_id: string; dev_eui: string; last_seen: string; uplink_count: number };
  });
}

export function getLatestUplink(deviceId: string): UplinkRecord | null {
  const stmt = db.prepare(`
    SELECT * FROM uplinks
    WHERE device_id = ?
    ORDER BY received_at DESC
    LIMIT 1
  `);

  stmt.bind([deviceId]);

  if (stmt.step()) {
    const row = stmt.getAsObject() as unknown as UplinkRecord;
    stmt.free();
    return row;
  }

  stmt.free();
  return null;
}

export function getStats(): {
  totalUplinks: number;
  uniqueDevices: number;
  firstUplink: string | null;
  lastUplink: string | null;
} {
  const countResult = db.exec('SELECT COUNT(*) as count FROM uplinks');
  const devicesResult = db.exec('SELECT COUNT(DISTINCT device_id) as count FROM uplinks');
  const firstResult = db.exec('SELECT MIN(received_at) as first FROM uplinks');
  const lastResult = db.exec('SELECT MAX(received_at) as last FROM uplinks');

  return {
    totalUplinks: (countResult[0]?.values[0]?.[0] as number) ?? 0,
    uniqueDevices: (devicesResult[0]?.values[0]?.[0] as number) ?? 0,
    firstUplink: (firstResult[0]?.values[0]?.[0] as string) ?? null,
    lastUplink: (lastResult[0]?.values[0]?.[0] as string) ?? null,
  };
}

export function getDatabase(): Database {
  return db;
}
