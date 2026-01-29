# Changelog

All notable changes to this project will be documented in this file.

## [1.2.2-beta]

### Added

- **Lidar Self-Occlusion:** Added "Diameter" parameter to LiDAR configuration.
- **False Positive Prevention:** Neighboring lidars falling within a sensor's FOV now cast shadows based on their physical diameter instead of being treated as point obstacles or ignored. This prevents lidars from seeing each other as safety hazards.

## [1.2.1-beta]

### Changed

- **Gen Tab UI:** Refactored "Plan Auto-Gen" to a matrix layout. Motion types (Linear, Turn, In-Place) and "Patch Notches" are now configurable per load case instead of globally.
- Updated Help documentation to reflect the new generation layout.

## [1.2.0-beta]

### Changed

- **Major Shadowing Overhaul:** Replaced the approximate wedge-based shadow logic with a precise visibility-based algorithm. The new `get_shadow_wedge` computes the true shadow volume by extruding sensor-facing edges, correctly handling concave shapes, self-occlusion, and complex silhouettes without blind spots.
- Fixed closure scope bug where "Edit Poly" and "Show Wrt Lidar" controls affected the wrong tab.
- Fixed crash (NameError) and view refresh issue when deleting points in the Results editor.

## [1.1.1-beta]

### Fixed

- Refactored Results tab numeric output to match visualization transforms exactly.
- Filtered numeric output to show only the active sensor when "Show Wrt Lidar" is enabled.

## [1.1.0-beta]

### Added

- "Show Wrt Lidar" toggle in Results Tab for sensor-centric visualization.
- Visual enhancements for Results Tab (View Selector, base_link label).

### Changed

- Moved the export/import buttons to editor page.
- Redesigned Physics Config in Gen Tab (matrix layout for per-load parameters).
- Improved "Edit Poly" interaction and view refreshing.

### Fixed

- Fixed various crashes (GeometryCollection, Layout issues).

### Todo

- Consider the lidar as mask. (i.e. right shouldn't interfere with the left if placed parallel and vise versa)
- export as nanoscan format (export in results)
- introduce field sets and fields
- monitoring_case table generation and export as nanoscan format
- import custom fields as dxf
- Linear field should be a trapezoid.
