# Changelog

All notable changes to this project will be documented in this file.

## [1.1.2-beta]

### Fixed

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
