/** Express server: CORS, parts/sounds routes, serves client. */
import 'dotenv/config';
import path from 'path';
import { fileURLToPath } from 'url';
import fs from 'fs';
import express, { type Request, type Response, type NextFunction } from 'express';
import cors from 'cors';
import partsRoutes from './routes/parts.js';
import soundsRoutes from './routes/sounds.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

const app = express();
const PORT = parseInt(process.env.PORT || '3001', 10);
// Comma-separated for multiple origins (e.g. Vercel production + preview URLs)
const allowedOriginsList = process.env.CLIENT_URL
  ? process.env.CLIENT_URL.split(',').map((s) => s.trim()).filter(Boolean)
  : ['http://localhost:5173'];

const demoMode = false;

// Allow exact matches from CLIENT_URL, or any origin ending with .vercel.app (preview URLs)
function corsOrigin(origin: string | undefined, cb: (err: Error | null, allow?: boolean | string) => void) {
  if (!origin) return cb(null, true); // same-origin or non-browser
  if (allowedOriginsList.includes(origin)) return cb(null, true);
  if (origin.endsWith('.vercel.app')) return cb(null, true);
  cb(null, false);
}
app.use(cors({ origin: corsOrigin }));
app.use(express.json());

app.use('/api/parts', partsRoutes);
app.use('/api/sounds', soundsRoutes);

app.get('/api/health', (_req, res) => {
  res.json({ status: 'ok', timestamp: new Date().toISOString(), demoMode });
});
// Root /health for Railway and generic health checks
app.get('/health', (_req, res) => {
  res.json({ status: 'ok', timestamp: new Date().toISOString(), demoMode });
});

// Serve built client (single-server hosting). Check both dist/../public and cwd/public.
const publicDirCandidates = [
  path.join(__dirname, '..', 'public'),
  path.join(process.cwd(), 'public'),
];
const publicDir = publicDirCandidates.find((dir) => fs.existsSync(dir));

if (publicDir) {
  app.use(express.static(publicDir, { index: 'index.html' }));
  // SPA fallback: any non-API GET that didn't match a file → index.html
  app.get('*', (req, res, next) => {
    if (req.path.startsWith('/api')) return res.status(404).json({ error: 'Not found' });
    res.sendFile(path.join(publicDir, 'index.html'), (err) => {
      if (err) next(err);
    });
  });
} else {
  app.get('/', (_req, res) => {
    res.status(404).send(
      '<h1>404 – Client not built</h1><p>Build the client and copy to <code>server/public</code>: <code>cd client && npm run build && cp -r dist ../server/public</code></p>'
    );
  });
}

// Global error handler: log and return 500
app.use((err: Error, _req: Request, res: Response, _next: NextFunction) => {
  console.error('[Server Error]', err?.stack ?? err);
  res.status(500).json({ error: 'Internal server error' });
});

const HOST = process.env.HOST || '0.0.0.0';
app.listen(PORT, HOST, () => {
  console.log(`\n  Robot Viewer Server running on http://${HOST}:${PORT}`);
  console.log('  Server ready.\n');
});
