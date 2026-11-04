// Render Construction/<slug>/preview.png with three.js in headless Chromium.
//   cd 3d-models/preview && npm install three@0.160.0 playwright && node render_previews.js [filter]
// (only `three` is needed if Playwright is already installed globally.)
// Grey ground with a 1 m grid and a 1.8 m reference figure (same look as the Mining previews).
const http = require('http'), fs = require('fs'), path = require('path');
const { chromium } = require('playwright');
const HERE = __dirname, ROOT = path.resolve(HERE, '..');   // 3d-models/
const JOBS = [   // [glb, azimuth, elevation, zoom]
  ['Construction/crawler-dozer/crawler_dozer.glb', 35, 20, 1.15],
  ['Construction/compact-track-loader/compact_track_loader.glb', 35, 22, 1.0],
  ['Construction/mini-excavator/mini_excavator.glb', 35, 20, 1.15],
  ['Construction/pickup-truck/pickup_truck.glb', 35, 22, 1.0],
  ['Construction/dually-pickup-truck/dually_pickup_truck.glb', 35, 22, 1.0],
];
const srv = http.createServer((req, res) => {
  const p = decodeURIComponent(req.url.split('?')[0]);
  const f = p.startsWith('/three/') ? path.join(HERE, 'node_modules', p)
          : p === '/preview.html' ? path.join(HERE, 'preview.html') : path.join(ROOT, p);
  fs.readFile(f, (e, d) => {
    if (e) { res.writeHead(404); return res.end(); }
    res.writeHead(200, { 'Content-Type': f.endsWith('.js') ? 'text/javascript' : f.endsWith('.html') ? 'text/html' : 'application/octet-stream' });
    res.end(d);
  });
});
srv.listen(0, async () => {
  const port = srv.address().port;
  const b = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
  const only = process.argv[2] || '';
  for (const [m, az, el, zoom] of JOBS.filter(j => j[0].includes(only))) {
    const pg = await b.newPage({ viewport: { width: 1200, height: 800 } });
    await pg.goto(`http://127.0.0.1:${port}/preview.html?m=/${m}&az=${az}&el=${el}&zoom=${zoom}`);
    await pg.waitForFunction("document.title !== ''", null, { timeout: 300000 });
    if (await pg.title() !== 'done') console.log(m, await pg.title());
    const out = path.join(ROOT, path.dirname(m), 'preview.png');
    await pg.screenshot({ path: out }); await pg.close(); console.log('wrote', path.relative(ROOT, out));
  }
  await b.close(); srv.close();
});
