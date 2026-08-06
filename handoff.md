# Project Context: Non-Planar Sawtooth Support + Pressure Recovery Tower
*(Custom fork of PrusaSlicer — libslic3r core)*

## Overview
This is a custom fork of PrusaSlicer, modifying `src/libslic3r/GCode.cpp` and `src/libslic3r/GCode.hpp` to change how support material is physically deposited, and to solve a nozzle pressure-loss problem when transitioning from support back to the model. Retraction tuning alone did not fix the pressure loss, so a physical solution was built instead.

## Part 1: Sawtooth Support
### Why it works
Standard flat-layer support was causing plastic waste issues. The fix replaces flat support extrusion with a non-planar sawtooth pattern: instead of printing each support layer as a flat plane, the pattern deposits material as a series of angled up-down teeth between a floor Z and an overshoot peak Z in the shape of a sawtooth. After that, there are squishing layers to ensure a flat surface for the next sawtooth later on.

### Where it lives
Inside `GCodeGenerator::_extrude()` in `GCode.cpp`, in a block marked:
```cpp
// --- BEGIN NON-PLANAR TOOLPATH INJECTION V12 ---
...
// --- END NON-PLANAR TOOLPATH INJECTION V12 ---
```
This sits after the acceleration/speed/e_per_mm setup code, and before the normal arc/line extrusion loop.

### Trigger Condition
Only applies when:
```cpp
path_attr.role == ExtrusionRole::SupportMaterial && bottom_z >= 0.0
```
This deliberately excludes `SupportMaterialInterface` — interface layers stay flat, since they need a clean flat surface for the model to sit on.

### Key Mechanics
- **`g_sawtooth_regions`**: A map keyed by quantized Z (in microns) storing `SawtoothRegion` polygons with a `floor_z`, used to detect whether the current support path overlaps a region that should be toothed.
- **`bottom_z`**: The floor of the current gap, found via bounding-box overlap between the current path and stored sawtooth regions.
- **`squish_z` / `true_peak_z`**: The layer's normal ("squish") plane vs. the overshoot peak the teeth reach, controlled by `SAW_TOOTH_OVERSHOOT` and clamped by `SAW_TOOTH_MIN_HEIGHT`.
- **Dynamic Tooth Height**: As the sawtooth support builds upward and approaches the flat interface layers, the height of the sawteeth dynamically shortens. This ensures they fit perfectly underneath the interface layers without poking through and ruining the flat surface meant for the model.
- For each path segment longer than 0.6mm, it's subdivided into teeth (~6mm wide). Each tooth = flat plateau -> downhill slope to `bottom_z` -> vertical rise back to `true_peak_z`, with a `G4` dwell (`SAW_TOP_SETTLE_MS`) after each vertical rise to let the pillar stiffen before the next sideways move.
- Extrusion volume math uses **HORIZONTAL PROJECTED DISTANCE**, not 3D segment length, to avoid over/under-extruding just because Z also changes.
- A single "anchor" vertical pillar prints once at the start of each path; only the **FINAL** pillar is closed back to the second-to-last peak (`previous_peak_point`), at reduced flow (`SAW_FINAL_CLOSE_FLOW_MULT = 0.50`), to avoid a long unsupported top traverse.

### Known Unresolved Issue
At the end of this block, `this->last_position = path.back().point` is set, but the writer's actual physical last position may be `previous_peak_point` (a different point in a different coordinate space — G-code mm vs. scaled slicer units). This mismatch has not been fully fixed yet, only flagged.

## Part 2: Pressure Recovery Tower
### Why it was added
Retraction tuning alone didn't fix nozzle pressure loss when transitioning from the toothed, up-and-down support extrusion printed slowly (3mm/s) back to the model printing fast (50mm/s). Rather than keep fighting retraction settings, the design goal became: build a real physical side tower that receives extra "priming" extrusion right after a sawtooth support pass, so the nozzle regains normal pressure on sacrificial plastic before it starts printing the model at normal speed/retraction.

### State Variables
Added to the `GCodeGenerator` class in `GCode.hpp` (NOT static locals, because one generator instance can be reused across exports):
```cpp
bool   m_saw_prime_after_support = false;
double m_saw_tower_last_built_z  = -1.0;
bool   m_saw_tower_brim_done     = false;
```

### Reset Location
These must be reset at the top of `GCodeGenerator::do_export()` in `GCode.cpp`, **NOT** in the constructor, because a single instance can run multiple exports.

### Flagging Support Completion
At the very end of the V12 sawtooth block (before `return gcode;`), added:
```cpp
m_saw_prime_after_support = true;
```
This arms the tower visit without printing it yet, since another support path might still follow on the same layer.

### Tower Logic Location
Inserted in `_extrude()` right after the `e_per_mm` calculation and before the speed-selection code. Classifies the current path role:
```cpp
saw_is_support  // SupportMaterial or SupportMaterialInterface
saw_is_model    // perimeter, infill, gap fill, ironing roles
```

Two conditions drive the tower:
1. `saw_build_tower`: Prints one square perimeter (currently 16x16mm, coords `SAW_TOWER_X/Y`) once per new Z layer.
2. `saw_prime_before_model`: Fires when `m_saw_prime_after_support` is true and the next path is a model role — prints short internal "prime" strokes, then clears the flag.

### Evolution of the Prime-Stroke Fix
1. **First version**: Two short inset lines (1.5mm from walls) — these floated in mid-air because they weren't anchored to any solid wall, and because the tower's last built perimeter was often ~2mm below the current recovery Z.
2. **Attempted fix**: Force a fresh perimeter on every recovery event — rejected because it broke the "one perimeter per Z" invariant needed for a continuous full-height tower.
3. **Final working approach**: Reverted to "one perimeter per layer," but changed the prime lines to intentionally overlap into the walls by 0.25mm (`PRIME_WALL_OVERLAP`) on both ends, so each line is a proper wall-to-wall anchored bridge instead of a floating stroke.

### Print Order Achieved
1. Sawtooth support
2. Tower wall (once per layer)
3. Prime strokes (anchored wall-to-wall)
4. Travel back to model start
5. Model prints at normal speed/retraction

## Part 3: Tower Brim
Added a one-time multi-loop brim around the tower's first printed layer for adhesion, using `m_saw_tower_brim_done` so it only prints once. Implemented as nested square loops from `SAW_TOWER_BRIM_WIDTH` (5mm) outward, spaced by `SAW_TOWER_BRIM_SPACING` (0.5mm), at `SAW_TOWER_BRIM_SPEED` (20mm/s).

### Bug Found and Fixed
The brim was printing at ~90mm/s (travel speed) because `m_writer.set_speed(brim_f, ...)` was called BEFORE `travel_to_xyz()`, and travel moves can overwrite the active feedrate.
**Fix**: Call `set_speed()` again immediately after each travel-to-brim-start and right before the first extrusion of that loop, inside the loop body — not just once outside it.

## Key Files Touched
- `src/libslic3r/NonPlanarConfig.h`
- `src/libslic3r/GCode.hpp`
  - Added `m_saw_prime_after_support`, `m_saw_tower_last_built_z`, `m_saw_tower_brim_done` member variables to `GCodeGenerator`.
- `src/libslic3r/GCode.cpp`
  - `_extrude()`: Added V12 sawtooth block (role == SupportMaterial) and tower/prime/brim block right after `e_per_mm` calculation.
  - `do_export()`: Added reset of the three new state variables at start of export.
