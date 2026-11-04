// Render Mining/<slug>/preview.png for every model with three.js in headless Chromium.
//   cd 3d-models/Mining/preview && npm install three@0.160.0 playwright && node render_previews.js
// (only `three` is needed if Playwright is already installed globally.)
// Same look as the Construction previews: grey ground with a 1 m grid, a 1.8 m reference figure.
const http = require('http'), fs = require('fs'), path = require('path');
const { chromium } = require('playwright');
const HERE = __dirname, ROOT = path.resolve(HERE, '..', '..');   // 3d-models/
const JOBS = [   // [glb, azimuth, elevation, zoom]
  ['Mining/large-mining-dozer/large_mining_dozer.glb', 35, 22, 1.12],
  ['Mining/large-wheel-loader/large_wheel_loader.glb', 35, 20, 1.25],
  ['Mining/water-cart/water_cart.glb', 35, 20, 1.25],
  ['Mining/mining-motor-grader/mining_motor_grader.glb', -70, 14, 1.25],   // front-left: shows the moldboard and circle
  ['Mining/mine-light-vehicle/mine_light_vehicle.glb', 35, 16, 1.0],
  ['Mining/mine-site-props/mine_site_props.glb', 50, 24, 1.35],
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
