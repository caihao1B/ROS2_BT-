PRAGMA user_version=16;
CREATE TABLE IF NOT EXISTS prototypes(
 key TEXT PRIMARY KEY, map_version TEXT NOT NULL, model_version TEXT NOT NULL,
 feature_version TEXT NOT NULL, category TEXT NOT NULL, mode TEXT NOT NULL,
 features TEXT NOT NULL CHECK(length(features)<=4096),
 advice TEXT NOT NULL CHECK(advice='fresh_imm'),
 created REAL NOT NULL, expires REAL NOT NULL, quality REAL NOT NULL,
 last_used REAL NOT NULL, uses INTEGER NOT NULL DEFAULT 0,
 provenance TEXT NOT NULL CHECK(length(provenance)<=1024));
CREATE TABLE IF NOT EXISTS audit(
 seq INTEGER PRIMARY KEY, t REAL NOT NULL, reason TEXT NOT NULL, key TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS metadata(key TEXT PRIMARY KEY, value TEXT NOT NULL);
