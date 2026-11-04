// Render a preview.png the way the dozer previews look (1200 x 800, grid, 1.8 m figure).
//
//   npm pack three@0.160.0 && tar xzf three-0.160.0.tgz   # gives ./package (three.js), once
//   THREE_DIR=./package node 3d-models/Farm/render_preview.js Farm/round-baler/round_baler.glb out.png ["zoom=1.2&dir=1,0.5,1"]
//
// Paths are relative to 3d-models/. Needs Playwright with Chromium. Query options:
// dir=x,y,z camera direction, zoom (distance / model size), tgt=x,y,z, nohuman=1,
// pose=node:axis:deg;... (axis x|y|z rotates, tx|ty|tz translates) to try the rig.
const http = require('http'), fs = require('fs'), path = require('path');
const { chromium } = require('playwright');
const HERE = __dirname, ROOT = path.dirname(HERE);
const THREE = path.resolve(process.env.THREE_DIR || 'package');
const types = { '.js': 'text/javascript', '.html': 'text/html', '.glb': 'model/gltf-binary' };
const srv = http.createServer((req, res) => {
  const p = decodeURIComponent(new URL(req.url, 'http://x').pathname);
  const f = p.startsWith('/three/') ? path.join(THREE, p.slice(7))
    : p === '/page.html' ? path.join(HERE, 'render_preview.html') : path.join(ROOT, p);
  fs.readFile(f, (e, d) => {
    if (e) { res.writeHead(404); res.end(); return; }
    res.writeHead(200, { 'Content-Type': types[path.extname(f)] || 'application/octet-stream' }); res.end(d);
  });
});
srv.listen(0, async () => {
  const [glb, out, extra = ''] = process.argv.slice(2);
  const b = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
  const pg = await b.newPage({ viewport: { width: 1200, height: 800 } });
  await pg.goto(`http://127.0.0.1:${srv.address().port}/page.html?m=/${glb}&${extra}`);
  await pg.waitForFunction('document.title=="done"', null, { timeout: 180000 });
  await pg.screenshot({ path: out });
  await b.close(); srv.close();
});
