# 3D models

A pack of detailed, true-scale (1:1, metres) vehicle models, each a single self-contained
`.glb`, generated from plain Python (no Blender or other 3D software needed).

| Model | File | Size (L × W × H) | Notes |
|---|---|---|---|
| **Crawler dozer** (tracked bulldozer, ~20 t class) | `crawler-dozer/crawler_dozer.glb` | ≈ 6.1 × 3.5 × 3.6 m | Semi-U blade 3.40 × 1.30 m, 1.98 m track gauge, 560 mm shoes, 203 mm chain pitch, 3-shank ripper |
| **Wheel dozer** (rubber-tyred articulated bulldozer, ~30 t class) | `wheel-dozer/wheel_dozer.glb` | ≈ 8.0 × 4.1 × 4.3 m | Straight blade 4.00 × 1.55 m, 3.70 m wheelbase, 2.40 m tread, 29.5R25 tyres |
| **Backhoe loader** (4WD, ~8 t) | `backhoe-loader/backhoe_loader.glb` | ≈ 6.6 × 2.4 × 3.8 m | 2.30 m loader bucket, 610 mm backhoe bucket with teeth, boom stowed and stabilisers folded for travel; 12.5/80-18 front and 16.9-28 rear tyres |
| **Compact (mini) excavator** (zero tail swing, ~3.5 t) | `mini-excavator/mini_excavator.glb` | ≈ 3.6 × 1.75 × 2.5 m | 300 mm rubber tracks, 0.87 m tail swing radius, offset boom on swing post, 450 mm bucket, dozer blade, cab; arm and bucket curled for travel |
| **Compact track loader** (vertical lift, ~4.5 t) | `compact-track-loader/compact_track_loader.glb` | ≈ 3.4 × 1.98 × 2.1 m | 400 mm rubber tracks on raised-sprocket undercarriage, vertical-lift arms, quick-attach plate, 1.98 m bucket |
| **Wheel tractor scraper** (single engine, ~16 m³ bowl) | `wheel-tractor-scraper/wheel_tractor_scraper.glb` | ≈ 13.2 × 3.6 × 3.9 m | Gooseneck and draft arms, bowl with apron, ejector, cutting edge and stinger, rear push block; 33.25-29 tyres |
| **Articulated dump truck** (6x6 off-road, ~40 t class) | `articulated-dump-truck/articulated_dump_truck.glb` | ≈ 10.9 × 3.6 × 3.7 m | Articulation hitch, tandem bogie with walking beams, tipping body with tail chute and hoist cylinders; 875/65R29 tyres |
| **Pickup truck** (half-ton crew cab, 5.5 ft bed, 4x4) | `pickup-truck/pickup_truck.glb` | ≈ 6.0 × 2.03 (2.5 with mirrors) × 1.96 m | 3.68 m wheelbase, 275/65R18 all-terrain tyres on 6-spoke alloys, LED lighting, bed with wheelhouses, under-bed spare |
| **Dually pickup truck** (one-ton heavy duty crew cab, 8 ft bed) | `dually-pickup-truck/dually_pickup_truck.glb` | ≈ 6.8 × 2.44 (3.0 with tow mirrors) × 2.03 m | 4.29 m wheelbase, dual rear wheels with flared fenders, LT235/80R17 tyres, chrome bumpers, roof clearance lights, tow mirrors |

Heights are to the top of the cab or roof (or the stowed boom); antennas and exhausts can add a
little. All of these are generic designs based on typical dimensions for their class — no real
manufacturer names, model numbers or logos.

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

## File conventions (model pack)

* **One `.glb` per model**, glTF 2.0, textures embedded, no Draco or other compression.
* **Units:** metres at real-world scale. **+Y is up, every vehicle faces +X.** The origin is on
  the ground, centred under the vehicle.
* **Rest pose:** wheels straight, dump body down, tailgate closed, booms stowed (the backhoe
  and mini excavator are in travel position, stabilisers folded up).

### Rigging

Every node and mesh has a descriptive name (`cab`, `frame`, `boom_tilt`, …). Moving parts
carry a suffix and their node origin sits on the real pivot; node rotations are identity in
the rest pose.

| Suffix | Motion | Used for |
|---|---|---|
| `_steer` | rotate about Y at the kingpin | front wheels of the backhoe and pickups |
| `_spin` | rotate about Z at the wheel centre | every wheel, track idler, sprocket and roller |
| `_tilt` / `_rotate` | hinge rotation about the pivot | blades, push frames, booms, dippers/arms, buckets, loader arms, ripper, dump body, scraper bowl and apron, tailgate, stabilisers; articulation hitches, excavator slew ring and swing post, linkages |
| `_slide` | translation | scraper ejector and every hydraulic cylinder rod |

Each hydraulic cylinder is `<name>_rotate` (barrel, origin at its base pin, child of the part
it is mounted on) with a child `<name>_rod_slide` (origin at the rod-eye pin). Parts that don't
move have no suffix.

### Materials, textures and UVs

* Materials use only these names: `paint_yellow`, `paint_blue`, `paint_white`,
  `paint_gray_dark`, `chassis_black`, `plastic_black`, `steel`, `steel_dark`, `track_steel`,
  `wear_steel`, `chrome`, `aluminum`, `tyre_rubber`, `hose_rubber`, `rubber_trim`,
  `seat_fabric`, `glass`, `glass_tinted`, `lamp_lens`, `lamp_amber`, `lamp_red`, `lamp_beacon`
  (each model uses the subset it needs, one material per surface type).
* Each material has a 1024² PNG `baseColorTexture` named after the material, holding its
  colour; `baseColorFactor` is white (only the glass materials keep an alpha value).
  Metal-roughness PBR only, no vertex colours, no extensions.
* Every primitive has `TEXCOORD_0`. UVs are unwrapped automatically: inside 0–1, no overlaps
  within each material's texture, one texel density for the whole model, and at least 4 px
  between islands (and from the texture edge) at 1024².

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

* `meshkit.py` — mesh library (boxes, cylinders, lathe, extrusions, belt paths), the rigged
  `Model` (nodes, pivots, rest pose, hydraulic cylinders, centring), materials and the `.glb` writer
* `uvkit.py` — automatic UV unwrapping and packing
* `dozer_parts.py` — shared assemblies (blade, cab, lamps, louvres, rails, exhaust)
* `bodykit.py` — smooth bodywork: lofted cross-sections, rounded boxes, flush glass/trim panels
* `vehicle_parts.py` — tyres (earthmover, industrial, all-terrain, highway), rims, rubber
  tracks, excavator and loader buckets, steering wheel
* `build_*.py` — one script per machine (`build_pickup_trucks.py` writes both pickups)
* `check_zfighting.py` — scans every model for z-fighting (coplanar, overlapping faces of
  different materials) and exits non-zero if it finds any: `python3 3d-models/check_zfighting.py`
* `validate_glb.py` — reads the generated `.glb` files and checks them against the model-pack
  rules above (format, orientation and origin, UVs, materials and textures, node naming and
  rig suffixes), exiting non-zero on any failure: `python3 3d-models/validate_glb.py`
