# 3D models

Detailed, true-scale (1:1, metres) 3D models of earthmoving equipment, generated from
plain Python (standard library only — no Blender or other dependencies).

| Model | Files | Size (L × W × H) | Notes |
|---|---|---|---|
| **Crawler dozer** (tracked bulldozer, ~20 t class) | `crawler-dozer/crawler_dozer.glb`, `.obj` + `.mtl` | ≈ 6.1 × 3.5 × 3.3 m (3.6 m to antenna tip) | Semi-U blade 3.40 × 1.30 m, 1.98 m track gauge, 560 mm shoes, 203 mm chain pitch, 3-shank ripper |
| **Wheel dozer** (rubber-tyred articulated bulldozer, ~30 t class) | `wheel-dozer/wheel_dozer.glb`, `.obj` + `.mtl` | ≈ 8.0 × 4.1 × 3.9 m (4.3 m to antenna tip) | Straight blade 4.00 × 1.55 m, 3.70 m wheelbase, 2.40 m tread, 29.5R25 tyres |

Both are generic designs based on typical dimensions for their class, not copies of any
manufacturer's machine.

![Crawler dozer](crawler-dozer/preview.png)
![Wheel dozer](wheel-dozer/preview.png)

## What's modelled

**Crawler dozer:** individual track shoes with grousers, chain links, bushings and shoe bolts
running a correct belt path round the idler and sprocket; 7 double-flange bottom rollers and
2 carrier rollers per side; toothed drive sprockets and final drives; track frames, roller
guards, recoil housings, equaliser bar and pivot shaft; engine hood with side louvres, front
radiator guard, exhaust and air pre-cleaner; semi-U blade with curved moldboard, cutting edge,
bolts, end bits, ribs and box beams; C-frame push arms on trunnions, tilt cylinder and brace,
twin lift cylinders with chrome rods and hoses; ROPS cab with glazing, seat, consoles,
joysticks, pedals, wipers, mirrors, work lights and beacon; fenders, fuel tank, handrails,
tail lights; parallelogram multi-shank ripper with tips and cylinders.

**Wheel dozer:** four tyres with L-3 style chevron lugs, rims, hubs and wheel nuts; axles with
differentials and planetary final drives; articulated front/rear frames with hitch pins and
steering cylinders; arched front fenders, rear walkway decks with tread plate, handrails and
access ladder; rear engine enclosure with louvres, rear radiator grille and counterweight;
exhaust and pre-cleaner; straight blade, push arms, lift and tilt cylinders; ROPS cab.

## File formats

* **`.glb`** (glTF 2.0 binary) — recommended. Opens in Blender, Windows 3D Viewer, three.js,
  Unity/Unreal importers, and online viewers such as gltf-viewer.donmccurdy.com. Each
  sub-assembly (blade, cab, tracks, wheels, …) is a separate named node.
* **`.obj` + `.mtl`** — for tools without glTF support. Keep the two files together.

Coordinates: **+Y up, +X forward**, origin on the ground under the machine's centre, 1 unit = 1 m.

## Viewing

Open `viewer.html` through a local web server (browsers block loading `.glb` files from
`file://`):

```bash
cd 3d-models
```

```bash
python3 -m http.server 8000
```

Then browse to http://localhost:8000/viewer.html. It has a 1 m grid, a 1.8 m reference
figure, a wireframe toggle and shows the model's overall dimensions.

## Regenerating

The models are built by code, so dimensions can be tweaked in the constants at the top of each
build script:

```bash
python3 3d-models/build_all.py
```

* `meshkit.py` — tiny mesh library (boxes, cylinders, lathe, extrusions, belt paths) with
  `.glb` and `.obj` writers
* `dozer_parts.py` — shared assemblies (blade, cab, lamps, louvres, rails, exhaust)
* `build_crawler_dozer.py`, `build_wheel_dozer.py` — the two machines
